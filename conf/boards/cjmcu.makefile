# Hey Emacs, this is a -*- makefile -*-
#
# cjmcu.makefile
#
# https://github.com/cleanflight/cleanflight/issues/22
# http://blog.oscarliang.net/build-fpv-micro-quadcopter-smallest-quad
# hw rev2 ?
#

BOARD=cjmcu
BOARD_CFG=\"boards/$(BOARD).h\"

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
$(TARGET).LDSCRIPT=$(SRC_ARCH)/cjmcu.ld

# -----------------------------------------------------------------------

# default flash mode is via SWD
# other possibilities: DFU-UTIL, JTAG, SWD, STLINK, SERIAL
FLASH_MODE ?= SWD

HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8002000
endif

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
AHRS_ALIGNER_LED   ?= 1
GPS_LED            ?= 2
SYS_TIME_LED       ?= 3


#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART1

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B38400

#
# default PPM input is on PA0
#
RADIO_CONTROL_PPM_PIN ?= PA0
ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_00 PA00 PA0))
  PPM_CONFIG=1
else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 PA01 PA1))
  PPM_CONFIG=2
else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_03 PA03 PA3))
  PPM_CONFIG=3
else
$(error Unknown RADIO_CONTROL_PPM_PIN, configure it to either PA00, PA01, PA03)
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
