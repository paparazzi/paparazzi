#!/bin/bash

# base address for Bebop
ADDR_BASE=192.168.42.1
PORT_BASE=5000

# test if a complete IP address is passed as first argument
if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
  PORT=$2
else
  ADDR=$ADDR_BASE
  PORT=$PORT_BASE
fi

pid=0

echo "Start video for $ADDR on port $PORT"
/usr/bin/avplay -loglevel quiet -max_delay 50 -fflags nobuffer rtp://$ADDR:$PORT

