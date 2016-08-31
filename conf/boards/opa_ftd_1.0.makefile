# Hey Emacs, this is a -*- makefile -*-
#
# Oversized Paparazzi (AP) Board
#
# http://wiki.paparazziuav.org/wiki/OPA
#

BOARD=opa_ftd
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-mx.ld

# -----------------------------------------------------------------------

#
# default flash mode is via SWD (JTAG)
#
FLASH_MODE ?= SWD

#
# default LED configuration
#
SYS_TIME_LED       ?= 1
RADIO_CONTROL_LED  ?= 2
ARMING_LED         ?= 3
FBW_MODE_LED       ?= none

#
# default uart configuration
#
SPEKTRUM_UART_SAT1 ?= UART1
#SPEKTRUM_UART_SAT2 ?= UART5

MODEM_PORT ?= UART3
MODEM_BAUD ?= B19200

INTERMCU_PORT ?= UART2
INTERMCU_BAUD ?= B230400

#
# default actuator configuration
#
ACTUATORS ?= actuators_pwm
