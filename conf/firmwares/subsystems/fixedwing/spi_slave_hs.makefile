# Hey Emacs, this is a -*- makefile -*-

#generic spi driver
$(TARGET).CFLAGS += -DUSE_SPI -DSPI_SLAVE_HS

ifeq ($(TARGET), sim)
else

$(TARGET).srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_slave_hs_arch.c

endif
