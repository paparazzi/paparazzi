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
BOARD_CFG=\"boards/$(BOARD_DIR)/$(BOARD).h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios
MCU=cortex-m7

# FPU on F7
USE_FPU=hard
USE_FPU_OPT= -mfpu=fpv5-d16

#USE_LTO=yes

$(TARGET).CFLAGS += -DSTM32H7 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32H7xx/platform.mk
CHIBIOS_BOARD_LINKER = STM32H743xI.ld
CHIBIOS_BOARD_STARTUP = startup_stm32h7xx.mk

# ITCM flash is a special flash that allow faster operations
# At the moment it is not possible to flash the code in this mode using dfu-util
# but it should work with the BlackMagicProbe or STLINK
# By default, normal flash is used
ifeq ($(USE_ITCM),1)
$(TARGET).CFLAGS += -DUSE_ITCM=1
DFU_ADDR = 0x00200000
else
$(TARGET).CFLAGS += -DUSE_ITCM=0
DFU_ADDR = 0x08000000
endif

# In this case we dont have LUFTBOOT but PX4_BOOTLOADER, but in order
# to correctly initialize the interrupt vector we have to define that
# the board has LUFTBOOT
HAS_LUFTBOOT ?= 1
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
endif

##############################################################################
# Compiler settings
#

# default flash mode is the PX4 bootloader
# possibilities: DFU, SWD, PX4 bootloader
FLASH_MODE ?= PX4_BOOTLOADER
PX4_TARGET = "ap"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/cube_orange.prototype"
PX4_BL_PORT ?= "/dev/serial/by-id/usb-Hex_ProfiCNC_CubeOrangeRevB-BL_460049001851303230373534-if00"


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
# default IMU configuration
#
IMU_MPU_SPI_DEV ?= spi4
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE3

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

