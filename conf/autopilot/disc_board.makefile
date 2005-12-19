ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP -DFBW
ap.srcs = $(SRC_ARCH)/ppm.c $(SRC_ARCH)/adc_ap.c $(SRC_ARCH)/uart_ap.c $(SRC_ARCH)/esc.c main_fbw.c inter_mcu.c pid.c estimator.c if_calib.c nav.c main_ap.c mainloop.c main.c
