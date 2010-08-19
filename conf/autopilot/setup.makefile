#
# setup.makefile
#
#


CFG_SETUP=$(PAPARAZZI_SRC)/conf/autopilot/subsystems/SETUP


SRC_SETUP=.
SRC_SETUP_ARCH=$(SRC_SETUP)/$(ARCH)
SRC_SETUP_TEST=$(SRC_SETUP)/

SETUP_INC = -I$(SRC_SETUP) -I$(SRC_SETUP_ARCH)





# a test program to tunnel between both uart

tunnel.CFLAGS += -DFBW -DBOARD_CONFIG=\"tiny_2_1_1_usb.h\" -DLED
tunnel.srcs += $(SRC_ARCH)/uart_tunnel.c


