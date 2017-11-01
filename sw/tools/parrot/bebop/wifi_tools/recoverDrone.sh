#!/bin/bash

if [ -z "$1" ]
then
IP="192.168.42.1"
echo "No drone ID specified, using ($IP)"
else
IP="$1"
echo "Drone ID specified, using ($IP)"
fi

{ echo "mount -o remount,rw /"; echo "sed -i 's|^/data/ftp/internal_000/scripts/connect2hub \& exit 0|exit 0|' /etc/init.d/rcS"; echo "chmod a+x /etc/init.d/rcS";  echo "/sbin/reboot"; sleep 10; } | telnet $IP
