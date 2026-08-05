#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

CAN_INTERFACE="${CAN_INTERFACE:-can0}"

exec sudo candump -L ${CAN_INTERFACE} > "${LOG_PATH}/actuation.can"
