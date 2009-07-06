# baro scp bits
ap.CFLAGS += -DUSE_BARO_SCP -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0 
ap.srcs += $(SRC_CSC)/csc_baro.c spi.c $(SRC_ARCH)/spi_hw.c

