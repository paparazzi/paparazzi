ARCHI=avr

ap.ARCHDIR = $(ARCHI)
ap.ARCH = atmega128
ap.TARGET = autopilot
ap.TARGETDIR = autopilot
ap.LOW_FUSE  = a0
ap.HIGH_FUSE = 99
ap.EXT_FUSE  = ff
ap.LOCK_FUSE = ff
ap.CFLAGS += -DAP -DMODEM -DMCU_SPI_LINK
ap.srcs = gps_ubx.c gps.c main_ap.c $(SRC_ARCH)/modem_hw.c inter_mcu.c link_mcu.c $(SRC_ARCH)/link_mcu_ap.c $(SRC_ARCH)/spi_ap.c $(SRC_ARCH)/adc_hw.c infrared.c pid.c nav.c estimator.c mainloop.c cam.c sys_time.c main.c

fbw.ARCHDIR = $(ARCHI)
fbw.ARCH = atmega8
fbw.TARGET = fbw
fbw.TARGETDIR = fbw
fbw.LOW_FUSE  = 2e
fbw.HIGH_FUSE = cb
fbw.EXT_FUSE  = ff
fbw.LOCK_FUSE = ff
fbw.CFLAGS += -DFBW -DINTER_MCU -DMCU_SPI_LINK
fbw.srcs = inter_mcu.c $(SRC_ARCH)/spi_fbw.c $(SRC_ARCH)/adc_hw.c sys_time.c main_fbw.c main.c
