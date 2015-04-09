#!/bin/bash

# base address for ARDrone2
ADDR_BASE=192.168.1.

# tmp folder for sdp files (relative path by default)
SDP_DIR=${PAPARAZZI_HOME=../../../..}/var/sdp_tmp

# test if a complete IP address is passed as first argument or just the last digit
if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
else
  ADDR=$ADDR_BASE$1
fi

pid=0

function quit {
  echo "Stop video"
  if [ "$pid" -gt 0 ]
  then
    kill -9 $pid
  fi
  exit 0
}

# trap control+c to stop mplayer
trap quit SIGINT

# fetch sdp file on the ARDrone
mkdir -p $SDP_DIR/$ADDR
../ardrone2.py --host=$ADDR download_file $SDP_DIR/$ADDR/stream.sdp images

if [ ! -f $SDP_DIR/$ADDR/stream.sdp ];
then
  echo "Unable to download sdp file from $ADDR"
  exit 0
fi

# start mplayer and respawn if needed
echo "Start video"
while [ 1 ]
do
  /usr/bin/mplayer -really-quiet $SDP_DIR/$ADDR/stream.sdp&
  pid=$!
  wait $pid
  echo "Restart video"
done

