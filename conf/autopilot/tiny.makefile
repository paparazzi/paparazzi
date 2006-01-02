# Makefile for the Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DFBW -DCONFIG=\"config_tiny.h\" -DLED -DACTUATORS -DGPS -DUBX
ap.srcs = estimator.c gps_ubx.c gps.c $(SRC_ARCH)/uart.c $(SRC_ARCH)/servos.c $(SRC_ARCH)/armVIC.c sys_time.c main_fbw_2.c main_ap_2.c main.c 
