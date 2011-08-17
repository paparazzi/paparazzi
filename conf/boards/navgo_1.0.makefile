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


RADIO_CONTROL_LED  = 1

BARO_LED = 2

AHRS_ALIGNER_LED = 3

### default settings
ifndef GPS_PORT
GPS_PORT	= UART0
endif
ifndef GPS_BAUD
GPS_BAUD	= B38400
endif
GPS_LED     = 4

ifndef MODEM_PORT
MODEM_PORT	= UART1
endif
ifndef MODEM_BAUD
MODEM_BAUD 	= B57600
endif

ADC_GENERIC_NB_SAMPLES = 16

# All targets on the NavGo board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

