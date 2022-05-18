# Hey Emacs, this is a -*- makefile -*-
#
# px4fmu_5.0_chibios.makefile
#
# This is for the main MCU (STM32F767) on the PX4 board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=px4fmu
BOARD_VERSION=5.0
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
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
CHIBIOS_BOARD_PLATFORM = STM32F7xx/platform.mk
CHIBIOS_LINKER_DIR = $(PAPARAZZI_SRC)/sw/airborne/arch/chibios/
CHIBIOS_BOARD_LINKER = STM32F76xxI.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f7xx.mk

##############################################################################
# Compiler settings
#

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/px4fmu_5.0.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-3D_Robotics_PX4_BL_FMU_v5.x_0-if00"


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
# default IMU configuration
#
IMU_MPU_SPI_DEV ?= spi1
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE0

#
# default UART configuration (modem, gps, spektrum)
#
SBUS_PORT ?= UART6
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART6

#The modem serial on px4 is called serial 1, but connected to uart2 on the f4
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

#The GPS serial on px4 is called serial 3, but connected to uart4 on the f4
GPS_PORT ?= UART1
GPS_BAUD ?= B57600

#InterMCU port connected to the IO processor
INTERMCU_PORT ?= UART8
INTERMCU_BAUD ?= B230400

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

