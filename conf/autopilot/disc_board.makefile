ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DFBW -DCONFIG=\"config_discboard.h\"  
ap.srcs = inter_mcu.c $(SRC_ARCH)/adc_hw.c sys_time.c main_fbw.c main.c
# pid.c estimator.c if_calib.c nav.c main_ap.c mainloop.c main.c
