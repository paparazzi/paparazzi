# Makefile for the Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DFBW -DCONFIG=\"config_tiny.h\" -DLED
ap.srcs = main_fbw_2.c main_ap_2.c main.c
