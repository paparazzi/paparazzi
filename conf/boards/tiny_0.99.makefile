#
# tiny_0.99.makefile
#
# http://paparazzi.enac.fr/wiki/Tiny_v0.99
#


include $(PAPARAZZI_SRC)/conf/boards/tiny_2.11.makefile


BOARD=tiny
BOARD_VERSION=0.99

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

GPS_UART_NR	= 1
GPS_LED = none
MODEM_UART_NR = 0
