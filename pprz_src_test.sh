#! /bin/sh
if test -z "$PAPARAZZI_SRC"; then
    PAPARAZZI_SRC=/usr/share/paparazzi
    PAPARAZZI_BIN=$PAPARAZZI_SRC/bin
else
    PAPARAZZI_BIN=$PAPARAZZI_SRC/sw/tools
fi
