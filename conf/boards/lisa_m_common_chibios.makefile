# Hey Emacs, this is a -*- makefile -*-
#
# Common makefile defines for Lia 1.1, Lisa M 2.0, and Lisa M 2.1
# (ChibiOS version)
#
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

# FPU not present on F1
USE_FPU=no
HARD_FLOAT=no

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F1xx/platform.mk
CHIBIOS_BOARD_LINKER = STM32F107xC.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f1xx.mk

##############################################################################
# Compiler settings
#
MCU  = cortex-m3

# default flash mode is via usb dfu bootloader (luftboot)
# other possibilities: DFU-UTIL, JTAG, SWD, STLINK, SERIAL
FLASH_MODE ?= DFU

HAS_LUFTBOOT ?= 1
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
DFU_ADDR = 0x8002000
DFU_PRODUCT = Lisa/Lia
endif
