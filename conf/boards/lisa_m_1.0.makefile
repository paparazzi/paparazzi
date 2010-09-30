#
# lisa_m_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/User/LisaM
#

BOARD=lisa_m
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = JTAG
#FLASH_MODE = SERIAL
endif

#
#
# some default values shared between different firmwares
#
#

SYS_TIME_LED       = 2

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT = UART5
#RADIO_CONTROL_LED  = 5

ifndef MODEM_PORT
MODEM_PORT=UART2
endif
ifndef MODEM_BAUD
MODEM_BAUD=B57600
endif

#AHRS_ALIGNER_LED = 7

GPS_PORT=UART1
GPS_BAUD=B38400
#GPS_LED = 3

#
# this is the DRDY pin of a max1168 on a booz IMU
#
# v 1.0
#
MAX_1168_DRDY_PORT = _GPIOD
MAX_1168_DRDY_PORT_SOURCE = PortSourceGPIOD
# v1.1 
#MAX_1168_DRDY_PORT = GPIOB
