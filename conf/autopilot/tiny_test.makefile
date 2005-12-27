# Configuration for a Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP
ap.CFLAGS +=  -DGPS -DUBX
ap.srcs = inter_mcu.c pid.c estimator.c gps_ubx.c gps.c nav.c cam.c main_ap.c mainloop.c main.c $(SRC_ARCH)/uart.c $(SRC_ARCH)/armVIC.c
# ap.srcs += $(SRC_ARCH)/modem.c $(SRC_ARCH)/adc_ap.c $(SRC_ARCH)/uart_ap.c $(SRC_ARCH)/servo.c
