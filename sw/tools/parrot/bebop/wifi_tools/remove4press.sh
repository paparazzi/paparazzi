#!/bin/bash

if [ -z "$1" ]
then
IP="192.168.42.1"
echo "No drone ID specified, using ($IP)"
else
IP="$1"
echo "Drone ID specified, using ($IP)"
fi

wput -u ./pprz_swarmhub.conf ftp://$IP/internal_000/scripts/pprz.conf
wput -u ./scripts/config_network.script ftp://$IP/internal_000/scripts/config_network.script
wput -u ./scripts/connect2hub ftp://$IP/internal_000/scripts/connect2hub
{ echo "mount -o remount,rw /";  echo "sed -i 's|^exit 0|/data/ftp/internal_000/scripts/connect2hub \& exit 0|' /etc/init.d/rcS"; echo "chmod a+x /etc/init.d/rcS"; echo "chmod a+x /data/ftp/internal_000/scripts/connect2hub"; echo "chmod a+x /data/ftp/internal_000/scripts/config_network.script"; echo "dos2unix /data/ftp/internal_000/scripts/pprz.conf"; echo "/sbin/reboot"; sleep 10; } | telnet $IP
