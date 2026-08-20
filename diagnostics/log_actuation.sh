#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

PREFIX="${2:-actuation}" # routine name from the config; one file, no suffix

CAN_INTERFACE="${CAN_INTERFACE:-can0}"

exec sudo candump -L ${CAN_INTERFACE} > "${LOG_PATH}/${PREFIX}.can"
