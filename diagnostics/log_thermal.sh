#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# FLIR Boson FFC-trigger serial line, exposed as a USB CDC-ACM device. Update
# this to your unit's udev symlink (see src/drivers/flir_thermal_driver/README.md)
THERMAL_DEV="${THERMAL_DEV:-/dev/flir_boson_serial_XXXXX}"

exec sudo socat -u ${THERMAL_DEV},raw,echo=0 - > "${LOG_PATH}/thermal.raw"
