ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DFBW -DCONFIG=\"config_discboard.h\" -DTIMER3 -DTIMER1_TOP=0x400 -DRADIO_CONTROL -DACTUATORS=\"servos_esc_hw.h\"
ap.srcs = sys_time.c radio_control.c $(SRC_ARCH)/ppm_hw.c $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/servos_esc_hw.c commands.c main_fbw.c main.c
# inter_mcu.c pid.c estimator.c if_calib.c nav.c main_ap.c mainloop.c main.c
