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
ap.CFLAGS += -DFBW -DAP -DCONFIG=\"robostix.h\" -DLED -DGPS -DUBX -DINFRARED -DRADIO_CONTROL -DINTER_MCU -DRADIO_CONTROL  -DACTUATORS=\"servos_direct_hw.h\" -DGPS_LINK=Uart1
ap.srcs = sys_time.c $(SRC_ARCH)/adc_hw.c inter_mcu.c gps_ubx.c gps.c infrared.c pid.c nav.c estimator.c main_fbw.c main_ap.c main.c commands.c radio_control.c  $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/servos_direct_hw.c
