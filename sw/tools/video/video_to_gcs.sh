#!/bin/sh
# Forward the RTP/JPEG video of an aircraft (module video_rtp_stream) to the video panel of PprzGCS.
# PprzGCS only shows JPEG frames, all video processing is done here with GStreamer.
#
# usage: video_to_gcs.sh [-i rtp_port] [-o gcs_port] [-r rotation]
#   -i  RTP port of the stream (default 5000, the second camera of video_rtp_stream is 6000)
#   -o  UDP port of the PprzGCS video panel (default 5600, layout: <configure source="udp://:5600"/>)
#   -r  rotation: none, clockwise, counterclockwise, rotate-180 (default none)

IN=5000
OUT=5600
ROT=none
while getopts "i:o:r:" opt; do
  case $opt in
    i) IN=$OPTARG ;;
    o) OUT=$OPTARG ;;
    r) ROT=$OPTARG ;;
    *) sed -n '2,9p' "$0"; exit 1 ;;
  esac
done

SRC="udpsrc port=$IN caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000 ! rtpjpegdepay"
SINK="udpsink host=127.0.0.1 port=$OUT sync=false"

if [ "$ROT" = "none" ]; then
  # JPEG frames as they come, one per datagram
  exec gst-launch-1.0 $SRC ! $SINK
else
  exec gst-launch-1.0 $SRC ! jpegdec ! videoflip method=$ROT ! jpegenc quality=85 ! $SINK
fi
