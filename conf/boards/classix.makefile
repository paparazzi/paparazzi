#
# classix.makefile
#
# http://paparazzi.enac.fr/wiki/Classix
#

ARCH=lpc21


BOARD=classix
BOARD_VERSION=1.0

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif


LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


### default settings for classix
GPS_BAUD = B38400
GPS_LED = none
RADIO_CONTROL_LED = none

# All targets on the TINY board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

# Battery Voltage
fbw.CFLAGS +=  -DUSE_AD0

#
# default LED configuration
#
ifndef RADIO_CONTROL_LED
RADIO_CONTROL_LED  = none
endif

ifndef BARO_LED
BARO_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = none
endif

ifndef GPS_LED
GPS_LED = 2
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = none
endif



