#
# classix.makefile
#
# http://paparazzi.enac.fr/wiki/Classix
#

ARCH=lpc21
ARCHI=arm7


BOARD=classix
BOARD_VERSION=1.0

#BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

# TODO: update syntax
BOARD_CFG = \"classix.h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif


LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


### default settings for classix
GPS_BAUD	= B38400

# All targets on the TINY board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

# Battery Voltage
fbw.CFLAGS +=  -DUSE_AD0

