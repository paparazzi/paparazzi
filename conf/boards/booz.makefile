#
# booz.makefile
#
# http://paparazzi.enac.fr/wiki/Booz
#
ARCH=lpc21
ARCHI=arm7
BOARD_CFG = \"boards/booz2_v1_0.h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif