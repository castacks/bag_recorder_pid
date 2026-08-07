#!/usr/bin/env bash
set -uo pipefail
# log usb disconnect and current troubles

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

PREFIX="${2:-}" # laptop vs desktop so not overwrite
FILE_PREFIX="${PREFIX:+${PREFIX}_}"

# stdbuf -oL: dmesg/udevadm default to fully-buffered stdout once it's not a
# terminal (piped/redirected), so a handful of events from a single
# unplug/replug can sit in memory indefinitely instead of hitting disk.
stdbuf -oL dmesg -w -T 2>&1 | grep --line-buffered -iE "usb|over-current|overcurrent" > "${LOG_PATH}/${FILE_PREFIX}usb_dmesg.log" &

# filter udev echo to just plug/unplug
stdbuf -oL udevadm monitor --kernel --subsystem-match=usb > "${LOG_PATH}/${FILE_PREFIX}usb_udevadm.log" &

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    # kill script's children (dmesg|grep, udevadm)
    pkill -TERM -P $$ 2>/dev/null
    exit 0
}
trap cleanup INT TERM EXIT

wait
