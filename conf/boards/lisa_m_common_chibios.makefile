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

$(TARGET).CFLAGS += -DSTM32F1 -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F1xx/platform.mk
# CHIBIOS_BOARD_LINKER script depends on whether we use a bootloader or not, see below
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
$(TARGET).CFLAGS+=-DLUFTBOOT -DCORTEX_VTOR_INIT=0x00002000
CHIBIOS_BOARD_LINKER = STM32F107xC_luftboot.ld
else
CHIBIOS_BOARD_LINKER = STM32F107xC.ld
endif

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1

#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART1
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B38400


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm

