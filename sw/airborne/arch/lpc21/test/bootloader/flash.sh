#!/bin/sh
../lpc21iap/lpc21iap $*

if [ $? = 2 ]
  then
    ../lpc21iap/lpc21iap bl_ram.elf
    ../lpc21iap/lpc21iap $*
fi
