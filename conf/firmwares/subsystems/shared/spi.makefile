# Hey Emacs, this is a -*- makefile -*-

#generic spi driver
$(TARGET).CFLAGS += -DUSE_SPI

ap.srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
sim.srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
