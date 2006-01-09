ARCHI=avr

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = atmega8
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.LOW_FUSE  = 2e
fbw.HIGH_FUSE = cb
fbw.EXT_FUSE  = ff
fbw.LOCK_FUSE = ff
fbw.CFLAGS += -DFBW -DACTUATORS
fbw.srcs = $(SRC_ARCH)/servos_4017.c sys_time.c main_fbw_2.c main.c

LOCAL_CFLAGS += -DCONFIG=\"config_v1_2_1.h\"