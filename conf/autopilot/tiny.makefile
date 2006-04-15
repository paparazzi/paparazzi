# Makefile for the Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS +=  -DFBW -DCONFIG=\"config_tiny.h\"
ap.srcs = sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c main_fbw.c main.c
#ap.CFLAGS +=  -DAP  -DLED -DGPS -DUBX
#ap.srcs = main_ap_2.c
# estimator.c gps_ubx.c gps.c 
