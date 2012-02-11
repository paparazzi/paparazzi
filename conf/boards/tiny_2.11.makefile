#
# tiny_2.11.makefile
#
# http://paparazzi.enac.fr/wiki/Tiny_v2
#

ARCH=lpc21

BOARD=tiny
BOARD_VERSION=2.11

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
SYS_TIME_LED = none
endif

### default settings for tiny_2 and twog
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


ADC_IR_TOP = ADC_0
ADC_IR1 = ADC_1
ADC_IR2 = ADC_2
ADC_IR_NB_SAMPLES = 16
ADC_GYRO_NB_SAMPLES = 16

ADC_GENERIC_NB_SAMPLES = 16

# All targets on the TINY board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

ACTUATORS = actuators_4017
