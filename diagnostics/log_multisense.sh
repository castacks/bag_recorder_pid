#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

MULTISENSE_IP="${MULTISENSE_IP:-10.41.8.132}"
INTERFACE="${INTERFACE:-eno8}"

exec sudo tcpdump src ${MULTISENSE_IP} -ni ${INTERFACE} -w "${LOG_PATH}/multisense.pcap"
