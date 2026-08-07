#!/usr/bin/env bash
set -euo pipefail

LOG_PATH="${1:-tmp}"
mkdir -p "${LOG_PATH}"

# FLIR Boson FFC-trigger serial lines, exposed as USB CDC-ACM devices via the
# udev symlinks flir_ros_sync uses (see src/drivers/flir_thermal_driver/src/flir_ros_sync/config/*.yaml).
THERMAL_DEV_LEFT="${THERMAL_DEV_LEFT:-/dev/flir_boson_serial_322011}"
THERMAL_DEV_RIGHT="${THERMAL_DEV_RIGHT:-/dev/flir_boson_serial_322008}"

# Raw V4L2 frame dump of the same two cameras (not just the FFC control
# line). Streams whatever pixel format/resolution the device is currently
# set to -- check with `v4l2-ctl --device=<dev> --list-formats-ext` if the
# captured file doesn't look right, and set THERMAL_VIDEO_FMT to pin it
# explicitly (e.g. width=640,height=512,pixelformat=Y16).
THERMAL_VIDEO_LEFT="${THERMAL_VIDEO_LEFT:-/dev/flir_boson_video_322011}"
THERMAL_VIDEO_RIGHT="${THERMAL_VIDEO_RIGHT:-/dev/flir_boson_video_322008}"
THERMAL_VIDEO_FMT="${THERMAL_VIDEO_FMT:-}"

sudo socat -u ${THERMAL_DEV_LEFT},raw,echo=0 - > "${LOG_PATH}/thermal_left.raw" &
sudo socat -u ${THERMAL_DEV_RIGHT},raw,echo=0 - > "${LOG_PATH}/thermal_right.raw" &

v4l2-ctl --device="${THERMAL_VIDEO_LEFT}" ${THERMAL_VIDEO_FMT:+--set-fmt-video=${THERMAL_VIDEO_FMT}} --stream-mmap --stream-to="${LOG_PATH}/thermal_left_video.raw" &
v4l2-ctl --device="${THERMAL_VIDEO_RIGHT}" ${THERMAL_VIDEO_FMT:+--set-fmt-video=${THERMAL_VIDEO_FMT}} --stream-mmap --stream-to="${LOG_PATH}/thermal_right_video.raw" &

wait
