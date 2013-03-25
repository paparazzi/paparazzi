#
# navgo_1.0.makefile
#
# prototype for NavGo board
#

ARCH=lpc21

BOARD=navgo
BOARD_VERSION=1.0

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
AHRS_ALIGNER_LED  ?= 2
GPS_LED           ?= 4
SYS_TIME_LED      ?= 1


#
# default uart settings
#
MODEM_PORT ?= UART1
MODEM_BAUD ?= B57600

GPS_PORT ?= UART0
GPS_BAUD ?= B38400


ADC_GENERIC_NB_SAMPLES = 16

# All targets on the NavGo board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

