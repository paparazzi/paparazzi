# Hey Emacs, this is a -*- makefile -*-
#
# Lisa_MX_2.1_chibios.makefile
#
#

BOARD=lisa_mx
BOARD_VERSION=2.1
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F4
USE_FPU=hard

# See list of supported Tier 3 architectures at: https://forge.rust-lang.org/platform-support.html
RUST_ARCH = thumbv7em-none-eabihf

$(TARGET).CFLAGS += -DSTM32F4 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_BOARD_LINKER = STM32F407xG.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f4xx.mk

# bootloader for Lisa MX is available from https://github.com/podhrmic/aggieair-bootloader
HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
DFU_ADDR = 0x8004000
DFU_PRODUCT = Lisa/Lia
endif

##############################################################################
# Compiler settings
#
MCU  = cortex-m4

include $(PAPARAZZI_SRC)/conf/boards/lisa_mx_defaults.makefile
