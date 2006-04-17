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
ap.CFLAGS += -DFBW -DCONFIG=\"robostix.h\" -DLED
ap.srcs = sys_time.c $(SRC_ARCH)/adc_hw.c main_fbw.c main.c
# ap.srcs +=  main_ap.c
