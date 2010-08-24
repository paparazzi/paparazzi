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


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = autopilot
sim.TARGETDIR = autopilot

