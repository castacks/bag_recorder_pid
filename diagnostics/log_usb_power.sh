#!/usr/bin/env bash
set -uo pipefail
# log usb disconnect and current troubles

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# Routine name from the config (usb_power_laptop vs usb_power_desktop) leads
# every file this script writes; the trailing word says which stream it is.
PREFIX="${2:-usb_power}"

# dmesg -w fails when kernel.dmesg_restrict=1 (the Ubuntu default) unless
# root, and leaves a zero-byte log that looks exactly like "nothing was
# plugged or unplugged during the run". Probe what actually works rather than
# assuming: hosts where the kernel log is readable unprivileged need no sudo
# at all, and hosts without passwordless sudo still capture whatever they can
# instead of the whole routine dying.
SUDO=""
if [ "$(id -u)" -ne 0 ] && ! dmesg -T >/dev/null 2>&1; then
    if sudo -n dmesg -T >/dev/null 2>&1; then
        SUDO="sudo -n"
    fi
fi

STARTED=0

# NOTE: stderr is deliberately NOT folded into the grep pipeline here. It used
# to be (`dmesg -w -T 2>&1 | grep -iE "usb|..."`), which meant a failure like
# "dmesg: read kernel buffer failed: Operation not permitted" didn't match the
# filter and was silently discarded, leaving an empty log and no explanation.
# stdbuf -oL: dmesg/udevadm default to fully-buffered stdout once it's not a
# terminal (piped/redirected), so a handful of events from a single
# unplug/replug can sit in memory indefinitely instead of hitting disk.
if ${SUDO} dmesg -T >/dev/null 2>&1; then
    ${SUDO} stdbuf -oL dmesg -w -T \
        | grep --line-buffered -iE "usb|over-current|overcurrent" \
        > "${LOG_PATH}/${PREFIX}_dmesg.log" &
    STARTED=$((STARTED + 1))
else
    echo "[usb_power] ERROR: cannot read kernel log, skipping dmesg stream" >&2
    echo "[usb_power] (kernel.dmesg_restrict=1 and no passwordless sudo here)" >&2
fi

# udevadm ships in the 'udev' package, which is absent from the tartandriver
# container image. Skip the stream loudly instead of creating a file that
# stays empty forever.
if command -v udevadm >/dev/null 2>&1; then
    # filter udev echo to just plug/unplug
    ${SUDO} stdbuf -oL udevadm monitor --kernel --subsystem-match=usb \
        > "${LOG_PATH}/${PREFIX}_udevadm.log" &
    STARTED=$((STARTED + 1))
else
    echo "[usb_power] ERROR: udevadm not found, skipping udev monitor stream" >&2
    echo "[usb_power] install the 'udev' package to capture plug/unplug events" >&2
fi

if [ "${STARTED}" -eq 0 ]; then
    echo "[usb_power] ERROR: no capture streams could start" >&2
    exit 1
fi

CLEANED_UP=0
cleanup() {
    if [ "${CLEANED_UP}" -eq 1 ]; then
        return
    fi
    CLEANED_UP=1
    # kill script's children (dmesg|grep, udevadm). The sudo'd halves run as
    # root, so an unprivileged pkill can't touch them -- escalate for those.
    pkill -TERM -P $$ 2>/dev/null
    if [ -n "${SUDO}" ]; then
        ${SUDO} pkill -TERM -P $$ 2>/dev/null
    fi
    exit 0
}
trap cleanup INT TERM EXIT

wait
