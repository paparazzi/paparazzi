#
# booz.makefile
#
# http://paparazzi.enac.fr/wiki/Booz
#
ARCH=lpc21
ARCHI=arm7
BOARD_CFG = \"boards/booz_1.0.h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif

#
#
# some default values for current firmwares
#
#

RADIO_CONTROL_LED  = 1
RADIO_CONTROL_LINK = UART0

MODEM_PORT = UART1
MODEM_BAUD = B57600
