# Configuration for a Tiny board (1 arm7tdmi, 1 LEA-LA)

ARCHI=arm7

ap.ARCHDIR = $(ARCHI)
ap.ARCH = arm7tdmi
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.CFLAGS += -DAP
ap.CFLAGS += -DMODEM
# -DGPS -DUBX -DDOWNLINK
ap.srcs = inter_mcu.c pid.c estimator.c cam.c main_ap.c mainloop.c main.c $(SRC_ARCH)/uart.c $(SRC_ARCH)/armVIC.c
ap.srcs += nav.c $(SRC_ARCH)/modem.c
# ap.srcs += gps_ubx.c gps.c
# ap.srcs += $(SRC_ARCH)/modem.c $(SRC_ARCH)/adc_ap.c $(SRC_ARCH)/uart_ap.c $(SRC_ARCH)/servo.c
