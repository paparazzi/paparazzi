# Hey Emacs, this is a -*- makefile -*-
#
# naze32.makefile
#
# https://code.google.com/p/afrodevices/wiki/AfroFlight32
# hw rev4
#

BOARD=naze32
BOARD_VERSION=rev4
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/naze32.ld

# -----------------------------------------------------------------------

# default flash mode is via SERIAL (USB plug which is connected to cp210x converter)
# other possibilities: DFU-UTIL, JTAG, SWD, STLINK, SERIAL
FLASH_MODE ?= SERIAL

#
#
# some default values shared between different firmwares
#
#


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
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART2

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART2
GPS_BAUD ?= B38400

#
# default PPM input is on PA0 (RC_CH1)
# configuring it to PA7 will also enable servos 1 and 2
#
RADIO_CONTROL_PPM_PIN ?= PA0
ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_07 PA07 PA7 RC_CH6))
  PPM_CONFIG=2
else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_00 PA00 PA0 RC_CH1))
  PPM_CONFIG=1
else
$(error Unknown RADIO_CONTROL_PPM_PIN, configure it to either PA7 or PA0)
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
