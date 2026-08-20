#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

PREFIX="${2:-multisense}" # routine name from the config; one file, no suffix

MULTISENSE_IP="${MULTISENSE_IP:-10.66.171.21}"
INTERFACE="${INTERFACE:-enp8s0}" # change when debugging.

exec sudo tcpdump src ${MULTISENSE_IP} -ni ${INTERFACE} -w "${LOG_PATH}/${PREFIX}.pcap"
