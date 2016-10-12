# Hey Emacs, this is a -*- makefile -*-
#
# openpilot_revo_nano.makefile
#
# https://librepilot.atlassian.net/wiki/display/LPDOC/OpenPilot+Revolution+Nano
#

BOARD=openpilot_revo
BOARD_VERSION=nano
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/openpilot_revo_nano.ld

# -----------------------------------------------------------------------

# default flash mode is via DFU-UTIL (short the two SBL pads at power up to get into DFU mode)
# other possibilities: DFU, DFU-UTIL, SWD, STLINK
FLASH_MODE ?= DFU-UTIL



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
# on the revo nano:
# UART1 -> flexi port shared with I2C1
# UART2 -> main port

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART2

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
