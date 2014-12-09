#!/bin/bash

# base address for ARDrone2
ADDR_BASE=192.168.1.

# tmp folder for images files (relative path by default)
IMAGES_DIR=${PAPARAZZI_HOME=../../../..}/var/images_tmp

# test if a complete IP address is passed as first argument or just the last digit
if [ `grep -c '\.' <<< $1` == 1 ]
then
  ADDR=$1
else
  ADDR=$ADDR_BASE$1
fi

# download images folder from ARDrone
mkdir -p $IMAGES_DIR/$ADDR
../ardrone2.py --host=$ADDR download_dir $IMAGES_DIR/$ADDR images

