#
# tiny_0.99.makefile
#
# http://wiki.paparazziuav.org/wiki/Tiny_v0.99
#

ARCH=lpc21

BOARD=tiny
BOARD_VERSION=0.99

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

# default flash mode is via usb bootloader
FLASH_MODE ?= IAP

LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000

#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED          ?= none
AHRS_ALIGNER_LED  ?= none
GPS_LED           ?= none
SYS_TIME_LED      ?= none


#
# default uart settings
#
MODEM_PORT ?= UART0
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B38400


#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
# and by setting the correct "driver" attribute in servo section
# e.g. <servo driver="Ppm">
#
ACTUATORS ?= actuators_4015

# All targets on the TINY board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)
