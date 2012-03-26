#
# sdlog_1.0.makefile
#
# Paparazzi SD Logger
#


include $(PAPARAZZI_SRC)/conf/boards/tiny_2.11.makefile

BOARD=sdlog
BOARD_VERSION=1.0

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
