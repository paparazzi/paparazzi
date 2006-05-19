# Makefile for the Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot

test.ARCHDIR = $(ARCHI)
test.ARCH = arm7tdmi
test.TARGET = autopilot
test.TARGETDIR = autopilot

#LPC21ISP_BAUD = 115200
#LPC21ISP_XTAL = 14746

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000
