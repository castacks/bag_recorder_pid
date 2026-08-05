#!/usr/bin/env bash
set -uo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# The kernel logs a distinct message for over-current trips and enumeration
# failures from insufficient power, and disconnect/reconnect churn (another
# common symptom of marginal power delivery) shows up here too.
sudo dmesg -w -T 2>&1 | grep --line-buffered -iE "usb|over-current|overcurrent" > "${LOG_PATH}/usb_dmesg.log" &

# Structured add/remove/bind/unbind per USB device. --subsystem-match=usb
# and --kernel keep it to bus-level device events only -- skips the
# input/hid/hidraw/leds/wakeup child-device noise and the duplicate UDEV
# echo of each event, so a plug/unplug is a handful of lines, not hundreds.
sudo udevadm monitor --kernel --subsystem-match=usb > "${LOG_PATH}/usb_udevadm.log" &

wait
