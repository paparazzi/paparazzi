#generic spi driver
$(TARGET).CFLAGS += -DUSE_SPI

ap.srcs += spi.c $(SRC_ARCH)/spi_hw.c
sim.srcs += spi.c $(SRC_ARCH)/spi_hw.c
