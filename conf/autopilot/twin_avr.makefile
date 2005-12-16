ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP -DDOWNLINK
ap.srcs = gps_ubx.c gps.c main_ap.c main.c $(SRC_ARCH)/modem.c link_mcu.c $(SRC_ARCH)/link_mcu_ap.c $(SRC_ARCH)/spi_ap.c $(SRC_ARCH)/adc_ap.c infrared.c pid.c nav.c $(SRC_ARCH)/uart_ap.c estimator.c if_calib.c mainloop.c cam.c

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = atmega8
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.LOW_FUSE  = 2e
fbw.HIGH_FUSE = cb
fbw.EXT_FUSE  = ff
fbw.LOCK_FUSE = ff
fbw.CFLAGS += -DFBW
fbw.srcs = main.c main_fbw.c $(SRC_ARCH)/ppm.c $(SRC_ARCH)/servo.c $(SRC_ARCH)/spi_fbw.c $(SRC_ARCH)/uart_fbw.c $(SRC_ARCH)/adc_fbw.c
