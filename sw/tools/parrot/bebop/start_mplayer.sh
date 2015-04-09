#!/bin/bash

# base address for Bebop
ADDR_BASE=192.168.42.1

# tmp folder for sdp files (relative path by default)
SDP_DIR=${PAPARAZZI_HOME=../../../..}/var/sdp_tmp

# test if a complete IP address is passed as first argument
if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
else
  ADDR=$ADDR_BASE
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
../bebop.py --host=$ADDR download_file $SDP_DIR/$ADDR/stream.sdp internal_000/images

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

