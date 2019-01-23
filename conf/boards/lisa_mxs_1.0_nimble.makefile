# Hey Emacs, this is a -*- makefile -*-
#
# lisa_mxs_1.0_nimble.makefile
#
# MXS 1.0 board setup for DelFly Nimble 
# https://wiki.paparazziuav.org/wiki/Lisa/MXS_v1.0
#

BOARD=lisa_mxs
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-mx.ld

# -----------------------------------------------------------------------

HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000
DFU_ADDR = 0x8004000
DFU_PRODUCT = Lisa/Lia
endif

ASPIRIN_2_SPI_DEV ?= spi2
ASPIRIN_2_SPI_SLAVE_IDX ?= SPI_SLAVE2

SDLOGGER_DIRECT_SPI ?= spi1
SDLOGGER_DIRECT_SPI_SLAVE ?= SPI_SLAVE1
HS_LOG_SPI_DEV ?= spi1
HS_LOG_SPI_SLAVE_IDX ?= SPI_SLAVE1 

RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1
LOGGER_LED         ?= 2

FLASH_MODE ?= SWD_NOPWR

# disabling PWM to prevent interference when PA3 remapped to ADC
$(TARGET).CFLAGS+=-DUSE_PWM5=0

include $(PAPARAZZI_SRC)/conf/boards/lisa_mx_defaults.makefile

