#
# hb_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/HB_v1
#
ARCH=lpc21
BOARD=hb
BOARD_VERSION=1.1
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif

#
#
# some default values shared between different firmwares
#
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
GPS_LED = 2
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = 1
endif

RADIO_CONTROL_LINK = UART0

ifndef MODEM_PORT
MODEM_PORT = UART0
endif
ifndef MODEM_BAUD
MODEM_BAUD = B57600
endif

ifndef GPS_PORT
GPS_PORT = UART1
endif
ifndef GPS_BAUD
GPS_BAUD = B38400
endif

ifndef ADC_IR1
ADC_IR1      = 1
ADC_IR1_CHAN = 0
endif
ifndef ADC_IR2
ADC_IR2      = 2
ADC_IR2_CHAN = 1
endif
ifndef ADC_IR3
ADC_IR_TOP      = 4
ADC_IR_TOP_CHAN = 3
endif
ifndef ADC_IR_NB_SAMPLES
ADC_IR_NB_SAMPLES = 16
endif

$(TARGET).ARCHDIR = $(ARCH)
