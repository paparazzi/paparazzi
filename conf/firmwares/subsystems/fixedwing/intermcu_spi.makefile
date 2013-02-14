# Hey Emacs, this is a -*- makefile -*-

# InterMCU type SPI

ifeq ($(INTER_MCU_SPI),)
INTER_MCU_SPI = SPI1
endif

ifeq ($(INTER_MCU_SLAVE),)
INTER_MCU_SLAVE = SLAVE0
endif

fbw.CFLAGS  += -DMCU_SPI_LINK -DUSE_$(INTER_MCU_SPI)_SLAVE -DSPI_SLAVE -DUSE_SPI
fbw.srcs    += $(SRC_FIXEDWING)/link_mcu_spi.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
ap_srcs     += $(SRC_FIRMWARE)/fbw_downlink.c
ap.CFLAGS   += -DMCU_SPI_LINK -DUSE_$(INTER_MCU_SPI) -DSPI_MASTER -DUSE_SPI_$(INTER_MCU_SLAVE) -DUSE_SPI
ap.srcs     += $(SRC_FIXEDWING)/link_mcu_spi.c mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_arch.c
SEPARATE_FBW    = 1
