#! /bin/sh
# (c) 2010 Paul Cox
# This script is GPLv3 licensed!
if [ "$(id -u)" != "0" ]; then
	echo "Sorry, you are not root."
	exit 1
fi

echo "Network Configuration Assistant"
echo "Enter wpa or wep"
read NETMODE

echo "Enter device name: (usually wlan0)"
read DEVNAME

echo "Enter MAC Address or d to use the default: (example: 00:1f:1f:76:6f:e2 )"
read MAC

ETCPATH=/mnt/etc
FPATH=$ETCPATH/rc5.d
#ETCPATH=$PWD
#PATH=$PWD
SCRIPTNAME=network_$DEVNAME\_$NETMODE
NETSCRIPT=$FPATH/$SCRIPTNAME

if [ "$MAC" = "d" ];then
 echo "sleep 2"                      1>$NETSCRIPT
else 
 echo "ifconfig wlan1 hw ether $MAC" 1>$NETSCRIPT
 echo "sleep 2"                      1>>$NETSCRIPT
fi 

echo "ifconfig $DEVNAME up" 1>>$NETSCRIPT
echo "sleep 2"              1>>$NETSCRIPT

echo "Enter SSID"
read SSID

if [ "$NETMODE" = "wpa" ]; then
 echo "Enter WPA Secret: (example: 0123401234 )"
 read SECRET
 WPASCONF=$ETCPATH/wpa_supplicant.conf
 echo "network={"         1>$WPASCONF
 echo "  ssid=\"$SSID\""  1>>$WPASCONF
 echo "  psk=\"$SECRET\"" 1>>$WPASCONF
 echo "}"                 1>>$WPASCONF
 echo "wpa_supplicant -Dwext -i$DEVNAME -c/etc/wpa_supplicant.conf &" 1>>$NETSCRIPT
 echo "sleep 2" 1>>$NETSCRIPT
else
 if [ "$NETMODE" = "wep" ]; then
  echo "Enter WEP Key: (example: 8e7a6e05499088f250ee2fb747 )"
  read KEY
  echo "iwconfig $DEVNAME essid $SSID"           1>>$NETSCRIPT
  echo "iwconfig $DEVNAME mode managed key $KEY" 1>>$NETSCRIPT
  echo "sleep 2"                                 1>>$NETSCRIPT
 else
  echo "Network type: $NETMODE not supported."
  exit 1
 fi
fi

echo "Enter static IP address or DHCP for dynamic"
read ADDR
if [ "$ADDR" = "DHCP" ]; then
 echo "dhclient $DEVNAME"       1>>$NETSCRIPT
else 
 echo "ifconfig $DEVNAME $ADDR" 1>>$NETSCRIPT
fi

chmod 777 $NETSCRIPT
ln -f -s $SCRIPTNAME $FPATH/S99network
exit 0
