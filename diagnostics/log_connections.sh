#!/usr/bin/env bash
set -uo pipefail

# Single snapshot of what's plugged into ports
LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# The routine name from the config (e.g. connections_laptop vs
# connections_desktop) is the whole filename -- one file, no suffix needed.
PREFIX="${2:-connections}"

OUT="${LOG_PATH}/${PREFIX}.log"

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
    # udevadm is what turns a bare /dev/ttyUSB* into an identifiable device.
    # Without it every lookup below comes back empty and every device gets
    # skipped, which reads as "nothing is plugged in" rather than "the tool
    # is missing" -- so say which one it is. (udevadm is absent from the
    # tartandriver container image; on a bare host it's there.)
    if ! command -v udevadm >/dev/null 2>&1; then
        echo "!! udevadm not found -- serial devices cannot be identified."
        echo "!! Install the 'udev' package. Bare device nodes present:"
        for dev in /dev/ttyUSB* /dev/ttyACM*; do
            [ -e "$dev" ] || continue
            echo "-- ${dev} -- (unidentified)"
        done
    else
        for dev in /dev/ttyUSB* /dev/ttyACM* /dev/ttyS*; do
            [ -e "$dev" ] || continue
            info=$(udevadm info -q property -n "$dev" 2>/dev/null | grep -E "ID_VENDOR|ID_MODEL|ID_SERIAL")
            # Bare platform ttyS* ports have no vendor/model/serial so nothing
            # real is attached, just motherboard UART headers. Skip them.
            [ -z "${info}" ] && continue
            echo "-- ${dev} --"
            echo "${info}"
            echo
        done
    fi

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

# Surface the degraded-snapshot case on the routine's own stderr too, so it's
# visible without opening the snapshot file.
if ! command -v udevadm >/dev/null 2>&1; then
    echo "[connections] WARNING: udevadm missing, serial devices unidentified in ${OUT}" >&2
fi

echo "[connections] wrote snapshot to ${OUT}"
