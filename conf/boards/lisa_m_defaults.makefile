# Hey Emacs, this is a -*- makefile -*-
#
# Defaults for Lia 1.1, Lisa M 2.0, and Lisa M 2.1
#

#
# default LED configuration
#
RADIO_CONTROL_LED  ?= 4
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= 2
GPS_LED            ?= 3
SYS_TIME_LED       ?= 1

#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   ?= UART1
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT ?= UART5

MODEM_PORT ?= UART2
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

