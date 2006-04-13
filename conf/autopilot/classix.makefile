# Makefile for the Classix board (2 arm7tdmi)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DCONFIG=\"config_classix.h\" -DLED
ap.srcs = $(SRC_ARCH)/armVIC.c sys_time.c main_ap_2.c main.c


fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = arm7tdmi
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.CFLAGS += -DFBW -DCONFIG=\"config_classix.h\" -DLED
fbw.srcs = sys_time.c main_fbw.c main.c
