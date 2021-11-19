# Hey Emacs, this is a -*- makefile -*-

ifndef SPI_INCLUDED

SPI_INCLUDED = 1

#generic spi master driver
SPI_CFLAGS = -DUSE_SPI -DSPI_MASTER
SPI_SRCS = mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

$(TARGET).CFLAGS += $(SPI_CFLAGS)
$(TARGET).srcs += $(SPI_SRCS)

endif
