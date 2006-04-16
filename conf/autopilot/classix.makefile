# Makefile for the Classix board (2 arm7tdmi)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DCONFIG=\"config_classix.h\" -DLED -DTIME_LED=1
ap.srcs = sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c main_ap_2.c main.c


fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = arm7tdmi
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.CFLAGS += -DFBW -DCONFIG=\"config_classix.h\" -DLED -DTIME_LED=1
fbw.srcs = sys_time.c $(SRC_ARCH)/sys_time_hw.c $(SRC_ARCH)/armVIC.c main_fbw.c main.c
