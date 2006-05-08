# Makefile for the Classix board (2 arm7tdmi)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot


fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = arm7tdmi
fbw.TARGET = fbw
fbw.TARGETDIR = fbw

test.ARCHDIR = $(ARCHI)
test.ARCH = arm7tdmi
test.TARGET = fbw
test.TARGETDIR = fbw

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000
