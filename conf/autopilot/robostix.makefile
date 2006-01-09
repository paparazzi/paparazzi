# Makefile for the Robostix board (1 avr mega128)

ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = 0xbf
ap.HIGH_FUSE = 0xc9
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP -DFBW -DCONFIG=\"config_robostix.h\" -DLED
ap.srcs = sys_time.c main_fbw_2.c main_ap_2.c main.c
