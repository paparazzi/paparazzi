# Hey Emacs, this is a -*- makefile -*-
#
# px4io_2.4.makefile
#
# This is for the main MCU (STM32F427) on the PX4 board
# See https://pixhawk.org/modules/pixhawk for details
#

BOARD=px4io
BOARD_VERSION=2.4
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/px4io_2.4.ld

# default flash mode is via usb dfu bootloader
# possibilities: DFU, SWD, PX4_BOOTLOADER
PX4_BL_PORT ?= "/dev/serial/by-id/usb-Paparazzi_UAV_CDC_Serial_STM32_*"
PX4_PROTOTYPE ?= "${PAPARAZZI_HOME}/sw/tools/px4/px4io-v2.prototype"
PX4_TARGET = "fbw"
FLASH_MODE ?= PX4_BOOTLOADER

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 1
FBW_MODE_LED	     ?= 3

#
# default UART configuration (modem, gps, spektrum)
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART1

#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
