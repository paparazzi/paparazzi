# Hey Emacs, this is a -*- makefile -*-
#
# apogee_1.0_chibios.makefile
#
#

BOARD=apogee
BOARD_VERSION=1.0
BOARD_DIR=$(BOARD)/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F4
USE_FPU=yes
HARD_FLOAT=yes

$(TARGET).CFLAGS += -DSTM32F4 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)
PROJECT_DIR=

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_BOARD_PORT = ARMCMx/STM32F4xx/port.mk
CHIBIOS_BOARD_LINKER = STM32F407xG_ccm.ld

##############################################################################
# Compiler settings
#
MCU  = cortex-m4

# default flash mode is via usb dfu bootloader
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL

HAS_LUFTBOOT = FALSE

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART2

SBUS_PORT ?= UART2

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm


## FIXME disable debug for now, no idea why it doesn't work
#$(TARGET).CFLAGS += \
#	-DCH_DBG_SYSTEM_STATE_CHECK=FALSE \
#	-DCH_DBG_ENABLE_CHECKS=FALSE \
#	-DCH_DBG_ENABLE_ASSERTS=FALSE \
#	-DCH_DBG_ENABLE_TRACE=FALSE \
#	-DCH_DBG_ENABLE_STACK_CHECK=FALSE \
#	-DCH_DBG_FILL_THREADS=FALSE \
#	-DCH_DBG_THREADS_PROFILING=FALSE
