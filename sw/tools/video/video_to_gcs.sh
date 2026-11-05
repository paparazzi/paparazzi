#!/bin/sh
# Show the RTP/JPEG video of an aircraft (module video_rtp_stream) in the video panel of PprzGCS,
# or in a GStreamer window of its own. PprzGCS only shows JPEG frames, all video processing is
# done here with GStreamer.

usage() {
  cat <<EOF
Show the RTP/JPEG video of an aircraft (module video_rtp_stream) in the PprzGCS video panel,
or in a GStreamer window of its own.

usage: $(basename "$0") [-i rtp_port] [-o gcs_port] [-r rotation] [-w] [-h]
  -i rtp_port   RTP port of the stream (default 5000; the second camera of video_rtp_stream is 6000)
  -o gcs_port   UDP port of the PprzGCS video panel (default 5600,
                layout: <widget name="video"><configure source="udp://:5600"/></widget>)
  -r rotation   none, clockwise, counterclockwise, rotate-180 (default none)
  -w            show the video in a GStreamer window instead of sending it to PprzGCS
  -h, --help    show this help

examples:
  $(basename "$0") -i 5000 -o 5600 -r counterclockwise   front camera of the course Bebop to PprzGCS
  $(basename "$0") -i 6000 -w                             bottom camera in a window
EOF
}

IN=5000
OUT=5600
ROT=none
WINDOW=0
[ "$1" = "--help" ] && { usage; exit 0; }
while getopts "i:o:r:wh" opt; do
  case $opt in
    i) IN=$OPTARG ;;
    o) OUT=$OPTARG ;;
    r) ROT=$OPTARG ;;
    w) WINDOW=1 ;;
    h) usage; exit 0 ;;
    *) usage; exit 1 ;;
  esac
done
shift $((OPTIND - 1))
if [ $# -gt 0 ]; then
  echo "unknown argument: $1" >&2
  usage
  exit 1
fi
case $ROT in
  none|clockwise|counterclockwise|rotate-180) ;;
  *) echo "unknown rotation: $ROT" >&2; usage; exit 1 ;;
esac

SRC="udpsrc port=$IN caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000 ! rtpjpegdepay"
SINK="udpsink host=127.0.0.1 port=$OUT sync=false"

if [ $WINDOW -eq 1 ]; then
  exec gst-launch-1.0 $SRC ! jpegdec ! videoflip method=$ROT ! videoconvert ! autovideosink sync=false
elif [ "$ROT" = "none" ]; then
  # JPEG frames as they come, one per datagram
  exec gst-launch-1.0 $SRC ! $SINK
else
  exec gst-launch-1.0 $SRC ! jpegdec ! videoflip method=$ROT ! jpegenc quality=85 ! $SINK
fi
