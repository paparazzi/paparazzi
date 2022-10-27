# Hey Emacs, this is a -*- makefile -*-
#
# cube_orange.makefile
#
# This is for the main MCU (STM32F767) on the PX4 board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=cube
BOARD_VERSION=orange
BOARD_DIR=$(BOARD)/$(BOARD_VERSION)
BOARD_CFG=\"arch/chibios/common_board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios
MCU=cortex-m7

# FPU on F7
USE_FPU=hard
USE_FPU_OPT= -mfpu=fpv5-d16

#USE_LTO=yes

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32H7xx/platform.mk
CHIBIOS_LINKER_DIR = $(PAPARAZZI_SRC)/sw/airborne/arch/chibios/
CHIBIOS_BOARD_LINKER = STM32H743xI.ld
CHIBIOS_BOARD_STARTUP = startup_stm32h7xx.mk

##############################################################################
# Compiler settings
#

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/cube_orange.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrange*-BL*"

#
# default LED configuration
#
SDLOG_LED          ?= none
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
# The TELEM2 port
SBUS_PORT ?= UART3
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART3

# The TELEM1 port (UART3 is TELEM2)
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

# The GPS1 port (UART8 is GPS2)
GPS_PORT ?= UART1
GPS_BAUD ?= B57600

# InterMCU port connected to the IO processor
INTERMCU_PORT ?= UART6
INTERMCU_BAUD ?= B1500000

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

