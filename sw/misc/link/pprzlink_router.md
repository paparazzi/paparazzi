####### GCS #########

adb root
adb push pprzlink_router_cc /data

adb shell
> su
> mount -o rw,remount /system
> echo "#!/system/bin/sh
/data/pprzlink_router_cc -e udp://192.168.0.11:5242:192.168.0.10:5243 -e udp://0.0.0.0:4243:192.168.43.255:4242 &" > /data/init.sh
> echo "#!/system/bin/sh
/data/init.sh &
/system/bin/ndc resolver setnetdns eth0 \"\" 8.8.8.8" > /system/bin/eth0_dns.sh
> chmod +x /data/init.sh
> chmod +x /data/pprzlink_router_cc
> chmod +x /system/bin/eth0_dns.sh
#####################

####### AIR #########
adb root
adb push pprzlink_router_cc /data
adb push mass_storage.sh /system/bin/mass_storage.sh

adb shell
> su
> mount -o rw,remount /system
> echo "#!/system/bin/sh
/data/pprzlink_router_cc -e uart:///dev/ttyS2:115200 -e udp://192.168.0.10:5243:192.168.0.11:5242 &" > /data/init.sh
> echo "[General]
#Mavlink-router serves on this TCP port
# TcpServerPort=5790
Log=/sdcard/flight_log/
ReportStats=false
MavlinkDialect=ardupilotmega

[LocalEndpoint boardendpoint:1]
SockName = boardendpoint
Binding = true

[LocalEndpoint cameraendpoint:1]
SockName = cameraendpoint
Binding = true

[UdpEndpoint gcs:2]
Mode = normal
Address = 192.168.0.11
Port = 14550" > /etc/mavlink-router.telepathy-air.conf
> chmod +x /data/init.sh
> chmod +x /data/pprzlink_router_cc
> chmod +x /system/bin/mass_storage.sh


#####################

echo "[General]
#Mavlink-router serves on this TCP port
# TcpServerPort=5790
Log=/sdcard/flight_log/
ReportStats=false
MavlinkDialect=ardupilotmega

[LocalEndpoint boardendpoint:1]
SockName = boardendpoint
Binding = true

[LocalEndpoint cameraendpoint:1]
SockName = cameraendpoint
Binding = true

[UdpEndpoint gcs:2]
Mode = normal
Address = 192.168.0.11
Port = 14550" > /etc/mavlink-router.telepathy-air.conf

[UartEndpoint flightcontrol:1]
Device = /dev/ttyS2
Baud = 57600,115200,921600,500000,1500000



mount -o rw,remount /system

echo "#!/system/bin/sh
/data/pprzlink_router_cc -e uart:///dev/ttyS2:115200 -e udp://192.168.0.10:5243:192.168.0.11:5242 &" > init.sh

echo "#!/system/bin/sh
/data/pprzlink_router_cc -e udp://192.168.0.11:5242:192.168.0.10:5243 -e udp://0.0.0.0:4243:192.168.43.255:4242 &" > init.sh



echo "#!/system/bin/sh
/data/init.sh &
/system/bin/ndc resolver setnetdns eth0 \"\" 8.8.8.8" > /system/bin/eth0_dns.sh

echo "#!/system/bin/sh
/data/init.sh &
# Copyright (c) 2009-2018, The Linux Foundation. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of The Linux Foundation nor
#       the names of its contributors may be used to endorse or promote
#       products derived from this software without specific prior written
#       permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
# NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
# OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
# OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
FILE=/data/.format_vfat.userdata
STORAGE=/dev/block/platform/soc/by-name/storage
TARGET=/sdcard
ATTR= \"-o iocharset=utf8,rw,sync,umask=0000,dmask=0000,fmask=0000\"
TYPE=\"-t vfat\"
if [ -f \"$FILE\" ]; then
	/bin/busybox mkfs.vfat $STORAGE
	rm $FILE
fi
/bin/busybox mount $TYPE $ATTR $STORAGE $TARGET" > /system/bin/mass_storage.sh