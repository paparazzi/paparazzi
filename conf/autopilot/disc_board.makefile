ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP -DFBW -DCONFIG=\"config_discboard.h\"
# ap.srcs = sys_time.c main_fbw_2.c main_ap_2.c main.c
ap.srcs = $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/adc_ap.c $(SRC_ARCH)/uart_ap.c $(SRC_ARCH)/servos_esc_hw.c main_fbw.c inter_mcu.c pid.c estimator.c if_calib.c nav.c main_ap.c mainloop.c main.c