#
# pc.makefile
#
# Linux PC
#

ARCH=sim
BOARD = pc
BOARD_VERSION=

BOARD_CFG = \"boards/pc_sim.h\"

$(TARGET).ARCHDIR = $(ARCH)

MODEM_HOST ?= 127.0.0.1

