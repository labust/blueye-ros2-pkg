#!/usr/bin/env bash
#
# Stream (and record) the Blueye ROV camera over RTSP using gstreamer
#
# Usage:
#   ./stream_blueye.sh [-H host] [-p port] [-m mountpoint] [-o output_file.mp4] [-d media_dir] [-f]
#
# Examples:
#   ./stream_blueye.sh                             # default 192.168.1.101:8554/test
#   ./stream_blueye.sh -o dive1.mp4                # display and record the raw H.264 stream to media/dive1.mp4
#   ./stream_blueye.sh -o dive1.mp4 -d recordings  # record into recordings/dive1.mp4 
#   ./stream_blueye.sh -f                          # display with an FPS overlay

set -euo pipefail

HOST="192.168.1.101"
PORT="8554"
MOUNT="test"
OUTPUT_FILE=""
MEDIA_DIR_NAME="media"
SHOW_FPS=""

while getopts "H:p:m:o:d:fh" opt; do
  case "$opt" in
    H) HOST="$OPTARG" ;;
    p) PORT="$OPTARG" ;;
    m) MOUNT="$OPTARG" ;;
    o) OUTPUT_FILE="$OPTARG" ;;
    d) MEDIA_DIR_NAME="$OPTARG" ;;
    f) SHOW_FPS="1" ;;
    h)
      grep '^#' "$0" | sed 's/^#//'
      exit 0
      ;;
    *)
      echo "Unknown option" >&2
      exit 1
      ;;
  esac
done

LOCATION="rtsp://${HOST}:${PORT}/${MOUNT}"

VIDEO_SINK="autovideosink sync=false"
if [[ -n "$SHOW_FPS" ]]; then
  VIDEO_SINK="fpsdisplaysink sync=false"
fi

trap '' INT

if [[ -n "$OUTPUT_FILE" ]]; then
  MEDIA_DIR="$(cd "$(dirname "$0")" && pwd)/${MEDIA_DIR_NAME}"
  mkdir -p "$MEDIA_DIR"
  OUTPUT_PATH="${MEDIA_DIR}/${OUTPUT_FILE}"

  echo "Streaming from ${LOCATION}, recording to ${OUTPUT_PATH}"
  gst-launch-1.0 rtspsrc location="${LOCATION}" latency=0 \
    ! rtph264depay ! h264parse config-interval=-1 ! tee name=t \
    t. ! queue ! avdec_h264 ! videoconvert ! ${VIDEO_SINK} \
    t. ! queue ! mp4mux fragment-duration=1000 streamable=true ! filesink location="${OUTPUT_PATH}" || true
else
  echo "Streaming from ${LOCATION}"
  gst-launch-1.0 rtspsrc location="${LOCATION}" latency=0 \
    ! rtph264depay ! avdec_h264 ! videoconvert ! ${VIDEO_SINK} || true
fi

echo "Stream stopped."
read -rp "Press Enter to close this terminal... "
