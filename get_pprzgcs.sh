#!/bin/sh

# check if the link already exists
if [ -f "var/pprzgcs" ]
then
    exit 0
fi

# if PprzGCS is installed on the system, use it
if command -v pprzgcs > /dev/null
then
    ln -s $(command -v pprzgcs) var/pprzgcs
    exit 0
fi

# PprzGCS not on the system. Get url to latest release.
PPRZGCS_URL=$(wget --quiet -O - https://api.github.com/repos/paparazzi/PprzGCS/releases/latest | grep browser_download_url | cut -d '"' -f 4)
filename=$(basename $PPRZGCS_URL)
# and download it
if [ ! -f "var/$filename" ]
then
    wget -nc -P var $PPRZGCS_URL
fi

# create link and make it executable
ln -f -s $filename var/pprzgcs
chmod +x var/pprzgcs

