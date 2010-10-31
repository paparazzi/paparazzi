#
# twog_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/Twog_v1
#

include $(PAPARAZZI_SRC)/conf/boards/tiny_2.11.makefile

BOARD=twog
BOARD_VERSION=1.0

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
