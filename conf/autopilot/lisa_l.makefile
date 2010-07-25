# Makefile for the Lisa board 

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

ARCH=stm32
ARCHI=stm32
SRC_ARCH=$(ARCH)
BOARD_CFG=\"boards/lisa_0.99.h\"
FLASH_MODE=JTAG

