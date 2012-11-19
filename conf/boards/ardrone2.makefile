# Hey Emacs, this is a -*- makefile -*-
#
# ardrone2.makefile
#
# http://paparazzi.enac.fr/wiki/AR.Drone_2_-_Specifications
#

BOARD=ardrone
BOARD_VERSION=2
BOARD_CFG=\"boards/$(BOARD)$(BOARD_VERSION).h\"

ARCH=omap_ardrone2
$(TARGET).ARCHDIR = $(ARCH)
#$(TARGET).MAKEFILE = omap_ardrone2

# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = FTP
endif

#
# some default values shared between different firmwares
#


#
# default LED configuration
#
ifndef RADIO_CONTROL_LED
RADIO_CONTROL_LED  = none
endif

ifndef BARO_LED
BARO_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = none
endif

ifndef GPS_LED
GPS_LED = none
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = none
endif

#
# default uart configuration
#
ifndef RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART1
endif

ifndef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT = UART5
endif

ifndef MODEM_PORT
MODEM_PORT=UART2
endif

ifndef MODEM_BAUD
MODEM_BAUD=B57600
endif

#FIXME: Port not yet determined
ifndef GPS_PORT
GPS_PORT=UART3
endif

ifndef GPS_BAUD
GPS_BAUD=B4800
endif

#
# Setting Default Actuators
#
ifndef ACTUATORS
ACTUATORS = actuators_pwm
endif
