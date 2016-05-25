# Hey Emacs, this is a -*- makefile -*-
#
# openpilot_revo_1.0.makefile
#
# https://www.openpilot.org/products/openpilot-revolution-platform/
#

BOARD=openpilot_revo
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/openpilot_revo.ld

# -----------------------------------------------------------------------

# default flash mode is via SWD
# other possibilities: DFU, DFU-UTIL, SWD, STLINK
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
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART2

MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
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
