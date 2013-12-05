# Hey Emacs, this is a -*- makefile -*-
#
# stm32f3_discovery.makefile
#
#

BOARD=stm32f3_discovery
BOARD_VERSION=
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=stm32
ARCH_L=f3
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/stm32f3discovery.ld #check!

HARD_FLOAT=yes

# default flash mode is via usb dfu bootloader
# possibilities: DFU, SWD
FLASH_MODE ?= n
STLINK ?= y
DFU_UTIL ?= n

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
SYS_TIME_LED       ?= 3

#
# default UART configuration (modem, gps, spektrum)
#

MODEM_PORT ?= UART6
MODEM_BAUD ?= B57600

GPS_PORT ?= UART4
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

