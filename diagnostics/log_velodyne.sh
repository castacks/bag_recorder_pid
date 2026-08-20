#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

PREFIX="${2:-velodyne}" # routine name from the config; one file, no suffix

VELODYNE1_IP="${VELODYNE1_IP:-192.168.3.201}"
VELODYNE2_IP="${VELODYNE2_IP:-192.168.3.202}"
INTERFACE="${INTERFACE:-eno1}"

exec tcpdump "src ${VELODYNE1_IP} or src ${VELODYNE2_IP}" -ni ${INTERFACE} -w "${LOG_PATH}/${PREFIX}.pcap"
