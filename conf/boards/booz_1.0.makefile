#
# booz_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/Booz
#
ARCH=lpc21
BOARD=booz
BOARD_VERSION=1.0
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
RADIO_CONTROL_LED  = 1
endif

ifndef BARO_LED
BARO_LED = 2
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = 3
endif

ifndef GPS_LED
GPS_LED = 4
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = 1
endif

RADIO_CONTROL_LINK = UART0


ifndef MODEM_PORT
MODEM_PORT = UART1
endif
ifndef MODEM_BAUD
MODEM_BAUD = B57600
endif

ifndef GPS_PORT
GPS_PORT=UART0
endif
ifndef GPS_BAUD
GPS_BAUD=B38400
endif
