# Hey Emacs, this is a -*- makefile -*-
#
# Lisa_MXS_1.0_chibios.makefile
#
#

BOARD=lisa_mxs
BOARD_VERSION=1.0
BOARD_DIR=$(BOARD)/chibios/v$(BOARD_VERSION)
BOARD_CFG=\"boards/$(BOARD_DIR)/board.h\"

ARCH=chibios
$(TARGET).ARCHDIR = $(ARCH)

RTOS=chibios

## FPU on F4
USE_FPU=hard

# See list of supported Tier 3 architectures at: https://forge.rust-lang.org/platform-support.html
RUST_ARCH = thumbv7em-none-eabihf

$(TARGET).CFLAGS += -DPPRZLINK_ENABLE_FD

##############################################################################
# Architecture or project specific options
#
# Define project name here (target)
PROJECT = $(TARGET)

# Project specific files and paths (see Makefile.chibios for details)
CHIBIOS_BOARD_PLATFORM = STM32F4xx/platform.mk
CHIBIOS_BOARD_LINKER = STM32F405xG.ld
CHIBIOS_BOARD_STARTUP = startup_stm32f4xx.mk

HAS_LUFTBOOT = FALSE

##############################################################################
# Compiler settings
#
MCU = cortex-m4

FLASH_MODE ?= SWD_NOPWR

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1
LOGGER_LED         ?= none

#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART2

SBUS_PORT ?= UART2

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B38400

#
# default PPM input is on PA01 (SERVO6)
#
RADIO_CONTROL_PPM_PIN ?= PA01
ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_10 PA10 UART1_RX))
  PPM_CONFIG=1
else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 PA01 PA1 SERVO6))
  PPM_CONFIG=2
else
$(error Unknown RADIO_CONTROL_PPM_PIN, configure it to either PA01 or PA10)
endif

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
