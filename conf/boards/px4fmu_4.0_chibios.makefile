# Hey Emacs, this is a -*- makefile -*-
#
# px4fmu_4.0_chibios.makefile
#
# This is for the main MCU (STM32F427) on the PX4 board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=px4fmu
BOARD_VERSION=4.0
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/$(BOARD).h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios
MCU=cortex-m4

# FPU on F4
USE_FPU=hard

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_LINKER_DIR = $(PAPARAZZI_SRC)/sw/airborne/arch/chibios/
CHIBIOS_BOARD_LINKER = STM32F427xT.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f4xx.mk

##############################################################################
# Compiler settings
#

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/px4fmu_4.0.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-3D_Robotics_PX4_BL_FMU_v4.x_0-if00"


#
# default LED configuration
#
SDLOG_LED          ?= 3
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 1
GPS_LED            ?= none
SYS_TIME_LED       ?= 2

#
# default UART configuration (modem, gps, spektrum)
#
SBUS_PORT ?= UART6
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART6

#The modem serial on px4 is called serial 1, but connected to uart2 on the f4
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

#The GPS serial on px4 is called serial 3, but connected to uart4 on the f4
GPS_PORT ?= UART4
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

