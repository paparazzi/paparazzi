# Hey Emacs, this is a -*- makefile -*-

# InterMCU type SPI


fbw.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_SLAVE
fbw.srcs 		+= $(SRC_FIXEDWING)/link_mcu_spi.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
ap_srcs			+= $(SRC_FIRMWARE)/fbw_downlink.c
ap.CFLAGS 		+= -DMCU_SPI_LINK -DUSE_SPI -DSPI_MASTER -DUSE_SPI_SLAVE0
ap.srcs 		+= $(SRC_FIXEDWING)/link_mcu_spi.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
SEPARATE_FBW		= 1
