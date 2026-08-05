#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

VELODYNE1_IP="${VELODYNE1_IP:-10.41.8.130}"
VELODYNE2_IP="${VELODYNE2_IP:-10.41.8.131}"
INTERFACE="${INTERFACE:-eno8}"

exec sudo tcpdump src ${VELODYNE1_IP} or ${VELODYNE2_IP} -ni ${INTERFACE} -w "${LOG_PATH}/velodyne.pcap"
