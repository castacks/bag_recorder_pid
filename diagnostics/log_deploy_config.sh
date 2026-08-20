#!/usr/bin/env bash
set -uo pipefail

# One-shot: copies the most recently written tmuxp_config.yaml (deploy.py's
# fully-resolved config -- actual commands/args/delays after merging in the
# registry and global params) into the bag's diagnostics dir. Relies on the
# same assumption as tmux capture: deploy.py runs before atvbag, so "most
# recent" is the one currently running. No matching stop_log needed.

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# Routine name from the config leads; the trailing word says which of the two
# configs it is: deploy_config_tmuxp.yaml, deploy_config_deploy.yaml
PREFIX="${2:-deploy_config}"

# Where deploy.py writes each run's data (the "data_folder" field in
# whichever deploy config was used). Override if yours differs.
DEPLOY_DATA_ROOT="${DEPLOY_DATA_ROOT:-/home/tartandriver/workspace/debug}"

latest=$(find "${DEPLOY_DATA_ROOT}" -maxdepth 2 -name tmuxp_config.yaml -printf '%T@ %p\n' 2>/dev/null | sort -rn | head -n1 | cut -d' ' -f2-)

if [ -z "${latest}" ]; then
    echo "[deploy_config] no tmuxp_config.yaml found under ${DEPLOY_DATA_ROOT}" >&2
    exit 0
fi

cp "${latest}" "${LOG_PATH}/${PREFIX}_tmuxp.yaml"
echo "[deploy_config] copied ${latest} -> ${LOG_PATH}/${PREFIX}_tmuxp.yaml"

# deploy.py writes deploy_config.yaml (the raw deploy config with its
# !include-d sub-configs expanded) alongside tmuxp_config.yaml in the same
# run directory.
deploy_cfg="$(dirname "${latest}")/deploy_config.yaml"
if [ -f "${deploy_cfg}" ]; then
    cp "${deploy_cfg}" "${LOG_PATH}/${PREFIX}_deploy.yaml"
    echo "[deploy_config] copied ${deploy_cfg} -> ${LOG_PATH}/${PREFIX}_deploy.yaml"
else
    echo "[deploy_config] no deploy_config.yaml found next to ${latest}" >&2
fi
