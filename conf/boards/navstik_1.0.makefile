# Hey Emacs, this is a -*- makefile -*-
#
# navstik_1.0.makefile
#
# http://wiki.paparazziuav.org/wiki/Navstik
#

BOARD=navstik
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).OOCD_INTERFACE=ftdi/ivygs
$(TARGET).OOCD_BOARD=navstik
$(TARGET).LDSCRIPT=$(SRC_ARCH)/navstik.ld

# -----------------------------------------------------------------------

# default flash mode is via usb dfu bootloader
# other possibilities: DFU-UTIL, JTAG
FLASH_MODE ?= JTAG

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= none
SYS_TIME_LED       ?= 1

#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART6

MODEM_PORT ?= UART5
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
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
