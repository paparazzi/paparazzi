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
ap.CFLAGS += -DFBW -DCONFIG=\"config_robostix.h\" -DLED -DRADIO_CONTROL -DACTUATORS=\"servos_direct_hw.h\" -DUART0
ap.srcs = sys_time.c radio_control.c  $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/servos_direct_hw.c commands.c main_fbw.c main.c
# ap.srcs +=  main_ap.c
