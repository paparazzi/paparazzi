# baro scp bits
ap.CFLAGS += -DUSE_BARO_SCP -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0
ap.srcs += $(SRC_CSC)/csc_baro.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c

