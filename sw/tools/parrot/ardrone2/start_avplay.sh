#!/bin/bash

# base address for ARDrone2
ADDR_BASE=192.168.1.

# test if a complete IP address is passed as first argument or just the last digit
if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
  PORT=$2
else
  ADDR=$ADDR_BASE$1
  PORT=$((5000+$1))
fi

pid=0

echo "Start video for $ADDR on port $PORT"
/usr/bin/avplay -loglevel quiet -max_delay 50 -fflags nobuffer rtp://$ADDR:$PORT

