include $(PAPARAZZI_SRC)/conf/autopilot/twin_avr.makefile
include $(PAPARAZZI_SRC)/conf/autopilot/twin_mcu.makefile

ap.srcs += $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/spi_hw.c
ap.CFLAGS += -DUSE_UART1 -DGPS_LINK=Uart1

fbw.CFLAGS += -DACTUATORS=\"servos_4017.h\" -DSERVOS_4017 -DADC
fbw.srcs += actuators.c $(SRC_ARCH)/adc_hw.c $(SRC_ARCH)/servos_4017.c $(SRC_ARCH)/ppm_hw.c  $(SRC_ARCH)/uart_hw.c $(SRC_ARCH)/spi_hw.c
