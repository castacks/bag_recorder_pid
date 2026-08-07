#!/usr/bin/env bash
set -uo pipefail

LOG_PATH="${1:-tmp}"
POLL_INTERVAL=5
mkdir -p "${LOG_PATH}"
# tmux pipe-pane's command runs in the target pane's own cwd, not this
# script's, so a relative LOG_PATH would write into the wrong place (or
# silently fail if that relative path doesn't exist there).
LOG_PATH="$(cd "${LOG_PATH}" && pwd)"

PREFIX="${2:-}" # laptop vs desktop so not overwrite
FILE_PREFIX="${PREFIX:+${PREFIX}_}"

# fine panes tagged by tartandriver_deploy launch key
list_panes() {
    tmux list-panes -a -F '#{session_name}|#{pane_id}|#{pane_title}' 2>/dev/null
}

# pane naming takes time during deploy. Skip and re-check next poll for
# any pane showing bare hostname. We only want to add the launch key titles,
# not hostname.
HOSTNAME="$(hostname)"

# tmux's pipe-pane -o is documented as "only open if not already piped",
# but empirically (tmux 3.2a) it toggles the pipe on/off on every call
# regardless of -o. So we track attached panes ourselves and only ever call
# pipe-pane once per pane, rather than relying on -o to no-op on repeats.
declare -A ATTACHED

attach_panes() {
    while IFS='|' read -r session pane_id title; do
        if [ -n "${ATTACHED[${pane_id}]+x}" ]; then
            continue
        fi
        title="${title:-untitled}"
        if [ "${title}" = "${HOSTNAME}" ]; then
            continue
        fi
        logfile="${LOG_PATH}/${FILE_PREFIX}${title}.log"
        # Some output printout formatting
        # -S -1000: last 1000 lines, -$(): strip trailing blank lines; -J: join
        # lines tmux soft-wrapped.
        backfill="$(tmux capture-pane -t "${pane_id}" -p -J -S -1000)"
        if [ -n "${backfill}" ]; then
            printf '%s\n' "${backfill}" | sed 's/[[:space:]]*$//' >> "${logfile}"
        fi
        tmux pipe-pane -t "${pane_id}" "cat >> '${logfile}'"
        ATTACHED[${pane_id}]=1
    done < <(list_panes)
}

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    for pane_id in "${!ATTACHED[@]}"; do
        tmux pipe-pane -t "${pane_id}" 2>/dev/null
    done
    # need exit 0 instead of trap so pipe and process dies
    exit 0
}
trap cleanup INT TERM EXIT

while true; do
    attach_panes
    sleep "${POLL_INTERVAL}"
done
