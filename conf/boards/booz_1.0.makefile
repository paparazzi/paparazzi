#
# booz_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/Booz
#
ARCH=lpc21
ARCHI=arm7
BOARD=booz
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif

#
#
# some default values shared between different firmwares
#
#

SYS_TIME_LED = 1

RADIO_CONTROL_LED  = 1
RADIO_CONTROL_LINK = UART0

MODEM_PORT = UART1
MODEM_BAUD = B57600
