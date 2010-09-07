#! /bin/sh
# (c) 2009 Graeme Gregory
# modified by Steve Sakoman and Paul Cox
# This script is GPLv3 licensed!

echo "SD card creator for OMAP"
if [ "$(id -u)" != "0" ]; then
	echo "Sorry, you are not root."
	exit 1
fi

echo "DANGEROUS: proceed carefully"

if [ "$2" = "extras" ] && [ -e extras.bz2 ];then
 EXTRAS=1
 echo "I will copy extras to rootfs."
else 
 EXTRAS=0
fi

if [ -z $1 ];then
  echo "Following is a list of the partitions in your computer"
  echo "---------------------------"
  cat /proc/partitions
  echo "---------------------------"
  echo "Please type the name of the SD device : (example: sdb)"
  read  DRIVE
 else 
  DRIVE=$1
fi

DRIVE=/dev/$DRIVE
echo "$DRIVE selected. Is this correct? [Type \"Y\" to proceed]"
read continue

if [ $continue = "Y" ]; then
 DRIVE1=${DRIVE}1
 DRIVE2=${DRIVE}2
 WORKDIR=$PWD

 echo "Checking if mounted..."
 umount $DRIVE1
 umount $DRIVE2

 echo "Configuring disk..."
 dd if=/dev/zero of=$DRIVE bs=1024 count=1024

 SIZE=`fdisk -l $DRIVE | grep Disk | awk '{print $5}'`

 echo DISK SIZE – $SIZE bytes
 
 CYLINDERS=`echo $SIZE/255/63/512 | bc`
 
 echo CYLINDERS – $CYLINDERS

 {
 echo ,9,0x0C,*
 echo ,,,-
 } | sfdisk -D -H 255 -S 63 -C $CYLINDERS $DRIVE

 echo "Checking if mounted..."
 umount $DRIVE1
 echo "formatting FAT partition..."
 mkfs.vfat -F 32 -n boot $DRIVE1

 echo "Checking if mounted..."
 umount $DRIVE2
 echo "formatting Linux partition..."
 mke2fs -j -L rootfs $DRIVE2 

 echo "Fetching files..."
 wget http://paparazzi.enac.fr/overo/latest/rootfs.bz2 http://paparazzi.enac.fr/overo/latest/uImage.bin

 echo -n "Writing files to FAT partition..."
 mount $DRIVE1 /mnt
 if [ $? -ne 0 ]; then
  echo "Mount failure!"
  exit 1
 fi
 cp MLO /mnt/
 cp uboot.bin /mnt/uboot
 cp uImage.bin /mnt/uImage
 umount /mnt
 echo "done."
 
 echo -n "Writing files to Linux partition..."
 mount $DRIVE2 /mnt
 if [ $? -ne 0 ]; then
  echo "Mount failure!"
  exit 1
 fi
 cd /mnt
 tar jxf $WORKDIR/rootfs.bz2
 if [ $EXTRAS -ne 0 ]; then
  mkdir /mnt/home/root/extras
  cd /mnt/home/root/extras
  tar jxf $WORKDIR/extras.bz2
 fi
 cd $WORKDIR
 umount /mnt
 echo "done."
fi
echo "Exiting."
exit 0
