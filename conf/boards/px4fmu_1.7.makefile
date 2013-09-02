# Hey Emacs, this is a -*- makefile -*-
#
# px4fmu_1.7.makefile
#
#

BOARD=px4fmu
BOARD_VERSION=1.7
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/apogee.ld

HARD_FLOAT=yes

# default flash mode is via usb dfu bootloader
# possibilities: DFU, SWD
FLASH_MODE ?= SWD


#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART6
GPS_BAUD ?= B38400

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART2


#
# default actuator configuration
#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_pwm
