# Hey Emacs, this is a -*- makefile -*-
#
# lisa_l_1.0.makefile
#
# http://wiki.paparazziuav.org/wiki/User/LisaL
#

BOARD=lisa_l
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-l.ld

# -----------------------------------------------------------------------
ifeq ($(BOARD_PROCESSOR),'omap')

	ARCH  = linux
	$(TARGET).LDFLAGS += -levent -lm

# -----------------------------------------------------------------------
else

	ARCH=stm32

	$(TARGET).ARCHDIR = $(ARCH)
# not needed?

endif
# -----------------------------------------------------------------------

# default flash mode is the onboard JTAG
FLASH_MODE ?= JTAG

#
#
# some default values shared between different firmwares
#
#

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 5
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 7
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1


#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
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
