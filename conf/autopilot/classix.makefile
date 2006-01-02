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
fbw.TARGET = autopilot
fbw.TARGETDIR = autopilot
fbw.CFLAGS += -DFBW -DCONFIG=\"config_classix.h\" -DLED
fbw.srcs = sys_time.c main_fbw_2.c main.c 
