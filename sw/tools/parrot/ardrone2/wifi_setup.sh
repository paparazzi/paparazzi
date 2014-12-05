#!/bin/sh
#
# Script to see if an IP adress is already used or not
#
# Getting SSID from config.ini file.

#initializing random generator
cat /data/random_mac.txt > /dev/urandom
/bin/random_mac > /data/random_mac.txt

#echo 2 > /proc/cpu/alignment

export NETIF=ath0
export WORKAREA="/lib/firmware"
export ATH_PLATFORM="parrot-omap-sdio"
export ATH_MODULE_ARGS="ifname=$NETIF"

# Switch Wifi mode depending on value into /data/config.ini
#

WIFI_MODE=`grep wifi_mode /data/config.ini | awk -F "=" '{ gsub(/ */,"",$2); print $2}'` 

case $WIFI_MODE in
0)
    WIFI_MODE=master
    ;;
1)
    WIFI_MODE=ad-hoc
    ;;
2)
    WIFI_MODE=managed
    ;;
*)
    WIFI_MODE=master
    ;;
esac


if [ -s /factory/mac_address.txt ]
then
MAC_ADDR=`cat /factory/mac_address.txt`
else
MAC_ADDR=`cat /data/random_mac.txt`
fi

loadAR6000.sh -i $NETIF --setmac $MAC_ADDR

# Waiting 2s for the wifi chip to be ready
sleep 2

AR6K_PID=`ps_procps -A -T -c | grep AR6K | awk '{print $1}'`
SDIO_PID=`ps_procps -A -T -c | grep ksdioirqd | awk '{print $1}'`

#Changing wifi priority
chrt -p -r 25 $SDIO_PID
chrt -p -r 24 $AR6K_PID


# Disabling powersaving
wmiconfig -i $NETIF --power maxperf
# Disabling 802.11n aggregation
wmiconfig -i $NETIF --allow_aggr 0 0
# enabling WMM
wmiconfig -i $NETIF --setwmm 1

i=0

while [ ! -n "$SSID" ] && [ $i -lt 10 ]
do
i=`expr $i + 1`
SSID=`grep ssid_single_player /data/config.ini | awk -F "=" '{print $2}'`

# Removing leading and trailing spaces
SSID=`echo $SSID`
sleep 1
done

if [ -n "$SSID" ]
then
echo "SSID=$SSID"
else
#default SSID.
SSID=ardrone2_wifi
echo "SSID=\"$SSID\""
fi

RANDOM_CHAN=auto


echo "Creating $WIFI_MODE Network $SSID"

iwconfig $NETIF mode $WIFI_MODE
iwconfig $NETIF essid "$SSID"

if [ "$WIFI_MODE" != "managed" ]
then
# Allowing ACS to select only channels 1 & 6
wmiconfig -i $NETIF --acsdisablehichannels 1
iwconfig $NETIF channel $RANDOM_CHAN
iwconfig $NETIF rate auto
iwconfig $NETIF commit
else
# The Wifi connection freezes when in managed mode if there is no keepalive
wmiconfig -i ath0 --setkeepalive 1
fi

wmiconfig -i $NETIF --dtim 1
wmiconfig -i $NETIF --commit

OK=0
BASE_ADRESS=`grep static_ip_address_base /data/config.ini | awk -F "=" '{print $2}'`
PROBE=`grep static_ip_address_probe /data/config.ini | awk -F "=" '{print $2}'`

# Removing leading and trailing spaces
BASE_ADRESS=`echo $BASE_ADRESS`
PROBE=`echo $PROBE`

# Default base address
if [ -n "$BASE_ADRESS" ]
then
echo "BASE_ADRESS=$BASE_ADRESS"
else
#default BASE_ADDRESS.
BASE_ADRESS=192.168.1.
echo "BASE_ADRESS=\"$BASE_ADRESS\""
fi

# Default probe
if [ -n "$PROBE" ]
then
echo "PROBE=$PROBE"
else
#default PROBE.
PROBE=1
echo "PROBE=\"$PROBE\""
fi

while [ $OK -eq 0 ]
do
#configuring interface.
ifconfig $NETIF $BASE_ADRESS$PROBE
arping -I $NETIF -q -f -D -w 2 $BASE_ADRESS$PROBE

if [ $? -eq 1 ]
then
	if [ -s /data/old_adress.txt ]
	then
		# Testing previously given adress.
		PROBE=`cat /data/old_adress.txt`
	else
		#generating random odd IP address
		PROBE=`/bin/random_ip`
	fi
	/bin/random_ip > /data/old_adress.txt
else
	echo $PROBE > /data/old_adress.txt
	OK=1
fi

done

#Configuring DHCP server.
echo "Using address $BASE_ADRESS$PROBE"
echo "start $BASE_ADRESS`expr $PROBE + 1`" > /tmp/udhcpd.conf
echo "end $BASE_ADRESS`expr $PROBE + 4`" >> /tmp/udhcpd.conf
echo "interface $NETIF" >> /tmp/udhcpd.conf
echo "decline_time 1" >> /tmp/udhcpd.conf
echo "conflict_time 1" >> /tmp/udhcpd.conf
echo "opt router  $BASE_ADRESS$PROBE" >> /tmp/udhcpd.conf
echo "opt subnet 255.255.255.0" >> /tmp/udhcpd.conf
echo "opt lease  1200" >> /tmp/udhcpd.conf

/bin/pairing_setup.sh

# Saving random info for initialization at next reboot
echo $MAC_ADDR `date` `/bin/random_mac` > /dev/urandom
/bin/random_mac > /data/random_mac.txt



telnetd -l /bin/sh

# Check if not booting in master mode
if [ "$WIFI_MODE" != "managed" ]
then
	udhcpd /tmp/udhcpd.conf
fi

# Adding route for multicast-packet
route add -net 224.0.0.0 netmask 240.0.0.0 dev $NETIF

# Allow reconnection to dirty TCP ports
echo "1" > /proc/sys/net/ipv4/tcp_tw_reuse 
echo "1" > /proc/sys/net/ipv4/tcp_tw_recycle 

# Starting the daemon which responds to the AUTH messages from FreeFlight if not in master mode
if [ "$WIFI_MODE" != "managed" ]
then
	/bin/parrotauthdaemon
fi

