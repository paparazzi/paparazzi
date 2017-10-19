# Hey Emacs, this is a -*- makefile -*-
#
# stm32f3_discovery_chibios.makefile
#
#

BOARD=stm32f3_discovery
BOARD_VERSION=1.0
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F3
USE_FPU=hard

$(TARGET).CFLAGS += -DSTM32F3 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F3xx/platform.mk
CHIBIOS_BOARD_LINKER = STM32F303xC.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f3xx.mk

##############################################################################
# Compiler settings
#
MCU  = cortex-m4


# default flash mode is via DFU-UTIL
# possibilities: DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL

HAS_LUFTBOOT = FALSE

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 5
GPS_LED            ?= 6
SYS_TIME_LED       ?= 3

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART3
SBUS_PORT ?= UART3


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
