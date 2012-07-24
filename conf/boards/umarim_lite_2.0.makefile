#
# umarim_lite_2.0.makefile
#
# Umarim Lite v2 board
#

ARCH=lpc21

BOARD=umarim
BOARD_VERSION=lite_2.0

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"


ifndef FLASH_MODE
FLASH_MODE = IAP
endif


LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


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
GPS_LED = 2
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = 1
endif


#
# default uart settings
#
ifndef GPS_PORT
GPS_PORT	= UART0
endif
ifndef GPS_BAUD
GPS_BAUD	= B38400
endif

ifndef MODEM_PORT
MODEM_PORT	= UART1
endif
ifndef MODEM_BAUD
MODEM_BAUD 	= B57600
endif



ADC_GENERIC_NB_SAMPLES = 16

#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
#
ifndef ACTUATORS
ACTUATORS = actuators_4017
endif


# All targets on the Umarim board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)
