#!/usr/bin/env bash
set -uo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# One line per pane: session_name<TAB>pane_id<TAB>pane_title. Panes are
# tagged with their tartandriver_deploy launch key as their title (see
# generate_tmuxp_config.py); untagged panes just get whatever title tmux
# assigned. Covers every session on the default tmux server, so multiple
# concurrent deploy sessions (e.g. perception_drivers + autonomy) are all
# captured in one pass.
mapfile -t PANES < <(tmux list-panes -a -F $'#{session_name}\t#{pane_id}\t#{pane_title}' 2>/dev/null)

if [ "${#PANES[@]}" -eq 0 ]; then
    echo "[log_tmux] no tmux sessions/panes found, nothing to capture" >&2
    sleep infinity &
    wait $!
    exit 0
fi

PANE_IDS=()
for entry in "${PANES[@]}"; do
    IFS=$'\t' read -r session pane_id title <<< "${entry}"
    title="${title:-untitled}"
    PANE_IDS+=("${pane_id}")
    # session name prefix keeps two concurrent sessions with a same-named
    # pane (e.g. both define a "static_tfs" launch entry) from colliding
    tmux pipe-pane -t "${pane_id}" -o "cat >> '${LOG_PATH}/${session}__${title}.log'"
done

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    for pane_id in "${PANE_IDS[@]}"; do
        tmux pipe-pane -t "${pane_id}" 2>/dev/null
    done
}
trap cleanup INT TERM EXIT

sleep infinity &
wait $!
