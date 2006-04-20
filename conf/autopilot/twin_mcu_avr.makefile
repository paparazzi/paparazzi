include $(PAPARAZZI_SRC)/conf/autopilot/twin_avr.makefile
include $(PAPARAZZI_SRC)/conf/autopilot/twin_mcu.makefile

ap.srcs += $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/link_mcu_ap.c $(SRC_ARCH)/spi_hw.c
ap.CFLAGS += -DGPS_LINK=uart1

fbw.CFLAGS += -DACTUATORS=\"servos_4017.h\" -DSERVOS_4017 -DADC
fbw.srcs += $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/servos_4017.c $(SRC_ARCH)/ppm_hw.c  $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/spi_hw.c
