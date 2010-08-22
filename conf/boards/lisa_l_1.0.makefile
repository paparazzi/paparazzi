#
# lisa_l_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/User/LisaL
#

ARCH=stm32
ARCHI=stm32
BOARD=lisa_l
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
ifndef FLASH_MODE
FLASH_MODE = JTAG
endif

#
#
# some default values shared between different firmwares
#
#

SYS_TIME_LED       = 1

RADIO_CONTROL_LED  = 5
RADIO_CONTROL_LINK = UART3

MODEM_PORT=UART2
MODEM_BAUD=B57600

