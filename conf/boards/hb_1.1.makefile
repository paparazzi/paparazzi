#
# hb_1.1.makefile
#
# http://wiki.paparazziuav.org/wiki/HB_v1
#
ARCH=lpc21
BOARD=hb
BOARD_VERSION=1.1
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

FLASH_MODE ?= IAP

#
#
# some default values shared between different firmwares
#
#

#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED          ?= none
AHRS_ALIGNER_LED  ?= none
GPS_LED           ?= 2
SYS_TIME_LED      ?= 1


RADIO_CONTROL_LINK = UART0

#
# default UART configuration
#
MODEM_PORT ?= UART0
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400


$(TARGET).ARCHDIR = $(ARCH)
