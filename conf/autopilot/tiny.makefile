# Configuration for a Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP -DFBW
ap.CFLAGS += -DUBX -DINFRARED -DGPS
ap.srcs = $(SRC_ARCH)/ppm.c main_fbw.c inter_mcu.c pid.c estimator.c if_calib.c infrared.c gps_ubx.c gps.c nav.c cam.c main_ap.c mainloop.c main.c
# ap.srcs += $(SRC_ARCH)/modem.c $(SRC_ARCH)/adc_ap.c $(SRC_ARCH)/uart_ap.c $(SRC_ARCH)/servo.c
