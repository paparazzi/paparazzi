# Hey Emacs, this is a -*- makefile -*-

ifndef SPI_INCLUDED

SPI_INCLUDED = 1

#generic spi driver
$(TARGET).CFLAGS += -DUSE_SPI -DSPI_MASTER

ap.srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
sim.srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

endif
