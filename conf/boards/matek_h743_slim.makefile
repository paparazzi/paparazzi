# Hey Emacs, this is a -*- makefile -*-
#
# matek_h743_slim.makefile
#
# This is for the Flight Controller Matek H743 SLIM
# See http://www.mateksys.com/?portfolio=h743-slim for details
#

BOARD=mateksys
BOARD_VERSION=FC-H743-SLIM
BOARD_DIR=$(BOARD)/$(BOARD_VERSION)
BOARD_CFG=\"arch/chibios/common_board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios
MCU=cortex-m7

# FPU on H7
USE_FPU=hard
USE_FPU_OPT= -mfpu=fpv5-d16

USE_LTO ?= yes

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32H7xx/platform.mk
CHIBIOS_LINKER_DIR = $(PAPARAZZI_SRC)/sw/airborne/arch/chibios/
CHIBIOS_BOARD_LINKER = STM32H743xI_no_bl.ld
CHIBIOS_BOARD_STARTUP = startup_stm32h7xx.mk

##############################################################################
# Compiler settings
#

# default flash mode is the DFU
# possibilities: DFU-UTIL, SWD, PX4 bootloader
FLASH_MODE ?= DFU-UTIL
DFU_ADDR = 0x08000000
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/matek_h743_slim.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-*_MatekH743_*"

#
# default LED configuration
#
SDLOG_LED          ?= none
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#
SBUS_PORT ?= UART6
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART6

MODEM_PORT ?= UART7
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B57600

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

