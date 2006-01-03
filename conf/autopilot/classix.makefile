# Makefile for the Classix board (2 arm7tdmi)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DCONFIG=\"config_classix.h\" -DLED -DGPS -DUBX
ap.srcs = estimator.c gps_ubx.c gps.c $(SRC_ARCH)/uart.c $(SRC_ARCH)/armVIC.c sys_time.c main_ap_2.c main.c 

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = arm7tdmi
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.CFLAGS += -DFBW -DCONFIG=\"config_classix.h\" -DLED -DRADIO_CONTROL
fbw.srcs = $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/armVIC.c sys_time.c main_fbw_2.c main.c 
