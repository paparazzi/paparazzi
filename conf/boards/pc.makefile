#
# pc.makefile
#
# Linux PC
#

ARCH=sim
ARCHI=sim
BOARD = pc
BOARD_VERSION=


# TODO: update board
BOARD_CFG = \"tiny.h\"


$(TARGET).ARCHDIR = $(ARCHI)
sim.ARCH = sitl

