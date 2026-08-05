#!/usr/bin/env bash
set -uo pipefail

# One-shot snapshot of what's plugged in where at the moment this runs.
# Unlike the other log_*.sh scripts this doesn't tail/block -- it writes one
# file and exits, so there's no matching stop_log needed for it.

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

OUT="${LOG_PATH}/connections.log"

{
    echo "=== snapshot taken: $(date -Is) ==="
    echo

    echo "=== USB tree (bus/port/device) ==="
    lsusb -t 2>&1
    echo

    echo "=== USB devices ==="
    lsusb 2>&1
    echo

    echo "=== Serial devices ==="
    for dev in /dev/ttyUSB* /dev/ttyACM* /dev/ttyS*; do
        [ -e "$dev" ] || continue
        info=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL")
        # Bare platform ttyS* ports have no vendor/model/serial -- nothing
        # real is attached, just motherboard UART headers. Skip them.
        [ -z "${info}" ] && continue
        echo "-- ${dev} --"
        echo "${info}"
        echo
    done

    echo "=== CAN interfaces ==="
    ip -details link show type can 2>&1
    echo

    echo "=== Ethernet interfaces ==="
    # -brief instead of -details, and drop docker/veth: physical ports only,
    # not container networking internals.
    ip -brief link show 2>&1 | grep -vE "^(lo|veth|docker)"
    echo

    echo "=== Neighbor table (what's reachable via which interface right now) ==="
    ip neigh 2>&1
} > "${OUT}" 2>&1

echo "[connections] wrote snapshot to ${OUT}"
