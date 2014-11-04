# Hey Emacs, this is a -*- makefile -*-

# InterMCU type SPI

# make sure that SEPARATE_FBW is configured
ifeq (,$(findstring $(SEPARATE_FBW),1 TRUE))
$(error Using intermcu via SPI, so dual mcu with separate fbw. Please add <configure name="SEPARATE_FBW" value="1"/>)
endif

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
