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

# Run a command on the remote host, optionally hopping into CONTAINER first
remote_exec() {
    local cmd="$1"
    if [ -n "${CONTAINER}" ]; then
        ssh "${HOST}" "docker exec ${CONTAINER} bash -c $(printf '%q' "${cmd}")"
    else
        ssh "${HOST}" "${cmd}"
    fi
}

remote_exec "mkdir -p '${REMOTE_DIR}'"
REMOTE_PID=$(remote_exec "setsid nohup '${REMOTE_DIAG_DIR}/${SCRIPT_NAME}' '${REMOTE_DIR}' </dev/null >/dev/null 2>&1 & echo \$!")

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1

    remote_exec "kill -INT -${REMOTE_PID}" 2>/dev/null
    for _ in 1 2 3 4 5; do
        remote_exec "kill -0 -${REMOTE_PID}" 2>/dev/null || break
        sleep 1
    done
    remote_exec "kill -0 -${REMOTE_PID}" 2>/dev/null && remote_exec "kill -KILL -${REMOTE_PID}" 2>/dev/null

    # Copy back whatever files the remote script produced, exactly once.
    # Through a container, pull via `docker exec ... tar` (no host-side bind
    # mount required); otherwise plain rsync over ssh.
    if [ -n "${CONTAINER}" ]; then
        ssh "${HOST}" "docker exec ${CONTAINER} tar -cf - -C '${REMOTE_DIR}' ." | tar -xf - -C "${LOCAL_LOG_DIR}"
    else
        rsync -a "${HOST}:${REMOTE_DIR}/" "${LOCAL_LOG_DIR}/"
    fi
    remote_exec "rm -r '${REMOTE_DIR}'"
}
# when stop_log sent, run cleanup function
trap cleanup INT TERM EXIT

# make sure the trap fires
sleep infinity &
wait $!
