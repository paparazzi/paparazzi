# Hey Emacs, this is a -*- makefile -*-

include $(PAPARAZZI_SRC)/conf/boards/lisa_l_1.0.makefile

#
# this is the DRDY pin of a max1168 on a booz IMU
#
# v1.1
#
MAX_1168_DRDY_PORT = _GPIOB
MAX_1168_DRDY_PORT_SOURCE = PortSourceGPIOB
