# Hey Emacs, this is a -*- makefile -*-

#generic spi driver
$(TARGET).CFLAGS += -DUSE_SPI


ifeq ($(ARCH), lpc21)
$(TARGET).CFLAGS += -DSSP_VIC_SLOT=9
else ifeq ($(ARCH), stm32)
endif

$(TARGET).srcs += mcu_periph/spi.c $(SRC_ARCH)/mcu_periph/spi_slave_hs_arch.c
