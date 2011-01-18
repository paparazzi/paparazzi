#
# tiny_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/Tiny_v1.1
#


include $(PAPARAZZI_SRC)/conf/boards/tiny_2.11.makefile


BOARD=tiny
BOARD_VERSION=1.1

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

GPS_PORT = UART1
GPS_LED = none
MODEM_PORT = UART0

ACTUATORS = actuators_4015
