#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

MULTISENSE_IP="${MULTISENSE_IP:-10.66.171.21}"
INTERFACE="${INTERFACE:-enx0826ae335659}" # change when debugging.

exec sudo tcpdump src ${MULTISENSE_IP} -ni ${INTERFACE} -w "${LOG_PATH}/multisense.pcap"
