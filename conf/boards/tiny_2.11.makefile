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
RADIO_CONTROL_LED ?= none
BARO_LED          ?= none
AHRS_ALIGNER_LED  ?= none
GPS_LED           ?= 2
SYS_TIME_LED      ?= none


#
# default uart settings
#
MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART0
GPS_BAUD ?= B38400


ADC_IR_TOP = ADC_0
ADC_IR1 = ADC_1
ADC_IR2 = ADC_2
ADC_IR_NB_SAMPLES = 16
ADC_GYRO_NB_SAMPLES = 16

ADC_GENERIC_NB_SAMPLES = 16

#
# you can use different actuators by adding a configure option to your firmware section
# e.g. <configure name="ACTUATORS" value="actuators_ppm/>
#
ifndef ACTUATORS
ACTUATORS = actuators_4017
endif


# All targets on the TINY board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)
