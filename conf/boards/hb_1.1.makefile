#
# hb_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/HB_v1
#
ARCH=lpc21
BOARD=hb
BOARD_VERSION=1.1
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif

#
#
# some default values shared between different firmwares
#
#

SYS_TIME_LED = 1 # not used on rotorcraft, only for tests

RADIO_CONTROL_LINK = UART0

ifndef MODEM_PORT
MODEM_PORT = UART0
endif
ifndef MODEM_BAUD
MODEM_BAUD = B57600
endif

ifndef GPS_PORT
GPS_PORT=UART1
endif
ifndef GPS_BAUD
GPS_BAUD=B38400
endif
