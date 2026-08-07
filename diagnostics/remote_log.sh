#!/usr/bin/env bash
set -uo pipefail

# Generic wrapper: runs a diagnostic script on a remote host (optionally
# inside a docker container there), blocks in the foreground, and on INT/TERM
# kills the remote process, copies back whatever it wrote, and cleans up the
# remote temp dir. Used by routines.diagnostics.start_log when a `host` is
# given, so any existing diagnostic script can be run remotely with no
# changes to itself.
# Usage: remote_log.sh <local_log_dir> <host> <name> <script_name> [container]
# (name/script_name resolution happens in routines.diagnostics, not here)

LOCAL_LOG_DIR="$1"
HOST="$2"
NAME="$3"
SCRIPT_NAME="$4"
CONTAINER="${5:-}"

mkdir -p "${LOCAL_LOG_DIR}"
ROUTINE_LOG_DIR="${LOCAL_LOG_DIR}/routine_logs"
mkdir -p "${ROUTINE_LOG_DIR}"

# Container path: if the target is only reachable via docker exec on the
# remote host (e.g. the deploy/tmux session lives inside a container there,
# not on the bare host), this can just be the container's own /tmp -- we pull
# files out via `docker exec ... tar` below, so it never needs to be visible
# from the bare host filesystem.
REMOTE_TMP_ROOT="${REMOTE_TMP_ROOT:-/tmp/tartandriver_diagnostics}"
# Update this to wherever bag_recorder_pid actually lives on the remote host
# (inside the container, if CONTAINER is set)
REMOTE_DIAG_DIR="${REMOTE_DIAG_DIR:-~/tartandriver_ws/src/core/bag_recorder_pid/diagnostics}"
REMOTE_DIR="${REMOTE_TMP_ROOT}/$(date +%Y%m%d_%H%M%S)_${NAME}"

# BatchMode disables interactive password/passphrase prompts (there's no TTY
# to answer them from here anyway, so fail fast instead of hanging), and
# accept-new auto-trusts hosts not yet in known_hosts without prompting.
SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o UpdateHostKeys=no)

# Run a command on the remote host, optionally hopping into CONTAINER first
remote_exec() {
    local cmd="$1"
    if [ -n "${CONTAINER}" ]; then
        ssh "${SSH_OPTS[@]}" "${HOST}" "docker exec ${CONTAINER} bash -c $(printf '%q' "${cmd}")"
    else
        ssh "${SSH_OPTS[@]}" "${HOST}" "${cmd}"
    fi
}

remote_exec "mkdir -p '${REMOTE_DIR}'"
remote_exec "setsid nohup ${REMOTE_DIAG_DIR}/${SCRIPT_NAME} '${REMOTE_DIR}' '${NAME}' </dev/null >'${REMOTE_DIR}/remote.log' 2>&1 &"

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1

    # Match by REMOTE_DIR (it's baked into the capture command's own args,
    # e.g. tcpdump's -w path) rather than the launch PID's process group --
    # more robust than tracking $!/pgid across whatever setsid/nohup/sudo
    # end up doing to the process tree. pgrep needs no special privilege to
    # just check for a match.
    #
    # Try unprivileged first (covers tools like tcpdump that run as atv via
    # setcap rather than sudo); only escalate to sudo if something's still
    # alive after that, for scripts that do need to run as root (candump,
    # socat on a root-only device, etc). A plain non-sudo kill/kill-0 on a
    # root-owned process silently fails with EPERM on both the signal *and*
    # the liveness check, which used to make this loop think the process
    # was already gone and race straight to rsync -- so don't skip the
    # unprivileged attempt's own wait loop before deciding to escalate.
    #
    # SIGTERM, not SIGINT: the remote process is launched as `cmd &` from a
    # non-interactive, non-job-control shell (plain `ssh host cmd`/`docker
    # exec ... bash -c`), and bash auto-sets SIGINT (and SIGQUIT) to ignored
    # for such async jobs, inherited straight through setsid/nohup/exec.
    # Verified directly against the real remote host: `kill -INT` on a
    # process launched this way is a silent no-op (exits 0, does nothing),
    # while `kill -TERM` on the same process works immediately. SIGTERM
    # isn't covered by that auto-ignore rule, and log_tmux.sh's own `trap
    # cleanup INT TERM EXIT` already handles it the same as INT.
    remote_exec "pkill -TERM -f '${REMOTE_DIR}'" 2>/dev/null
    for _ in 1 2 3 4 5; do
        remote_exec "pgrep -f '${REMOTE_DIR}'" >/dev/null 2>&1 || break
        sleep 1
    done
    if remote_exec "pgrep -f '${REMOTE_DIR}'" >/dev/null 2>&1; then
        remote_exec "sudo pkill -TERM -f '${REMOTE_DIR}'" 2>/dev/null
        sleep 1
        remote_exec "pgrep -f '${REMOTE_DIR}'" >/dev/null 2>&1 && remote_exec "sudo pkill -KILL -f '${REMOTE_DIR}'" 2>/dev/null
    fi

    # Copy back whatever files the remote script produced, exactly once.
    # Through a container, pull via `docker exec ... tar` (no host-side bind
    # mount required); otherwise plain rsync over ssh.
    if [ -n "${CONTAINER}" ]; then
        ssh "${HOST}" "docker exec ${CONTAINER} tar -cf - -C '${REMOTE_DIR}' ." | tar -xf - -C "${LOCAL_LOG_DIR}"
    else
        rsync -a -e "ssh ${SSH_OPTS[*]}" "${HOST}:${REMOTE_DIR}/" "${LOCAL_LOG_DIR}/"
    fi
    # remote_log.sh's own stdout/stderr capture (remote.log) lands flat in
    # LOCAL_LOG_DIR alongside the rest of the rsynced files -- move it into
    # its own namespaced spot so concurrent diagnostics sharing the same
    # LOCAL_LOG_DIR don't clobber each other's, and it stays visibly
    # separate from the actual diagnostic output.
    if [ -f "${LOCAL_LOG_DIR}/remote.log" ]; then
        mv "${LOCAL_LOG_DIR}/remote.log" "${ROUTINE_LOG_DIR}/${NAME}.log"
    fi
    remote_exec "rm -r '${REMOTE_DIR}'"
    exit 0
}
# when stop_log sent, run cleanup function
trap cleanup INT TERM EXIT

# Foreground, not `sleep infinity & wait $!`: a backgrounded job in a
# non-interactive shell gets SIGINT/SIGQUIT auto-ignored by bash, inherited
# through to the child, so it would survive this script exiting on signal --
# left behind as an orphan that still holds this process's inherited
# stdout/stderr open. Harmless on its own, but fatal to anything downstream
# waiting for that pipe to close (e.g. `| tee`), since it never will. A
# foreground sleep isn't subject to that auto-ignore rule and gets
# interrupted immediately like any other foreground command.
sleep infinity
