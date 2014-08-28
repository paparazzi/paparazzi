#
# booz_1.0.makefile
#
# http://wiki.paparazziuav.org/wiki/Booz
#
ARCH=lpc21
BOARD=booz
BOARD_VERSION=1.0
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
RADIO_CONTROL_LED  ?= 1
BARO_LED           ?= 2
AHRS_ALIGNER_LED   ?= 3
GPS_LED            ?= 4
SYS_TIME_LED       ?= none

RADIO_CONTROL_LINK = UART0


MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART0
GPS_BAUD ?= B38400

