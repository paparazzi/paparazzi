#
# hbmini_1.0.makefile
#
# prototype for HBMini board
#

ARCH=lpc21

BOARD=hbmini
BOARD_VERSION=1.0

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

FLASH_MODE ?= IAP


LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED          ?= none
AHRS_ALIGNER_LED  ?= none
GPS_LED           ?= none
SYS_TIME_LED      ?= none


#
# default uart settings
#
MODEM_PORT ?= UART0
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400

# All targets on the HBMini board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

