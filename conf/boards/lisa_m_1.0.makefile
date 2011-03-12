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
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny

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

SYS_TIME_LED       = 1

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

ifndef GPS_PORT
GPS_PORT=UART1
endif
ifndef GPS_BAUD
GPS_BAUD=B38400
endif
GPS_LED = none

#
# this is the DRDY pin of a max1168 on a booz IMU
#
# v 1.0
#
MAX_1168_DRDY_PORT = _GPIOD
MAX_1168_DRDY_PORT_SOURCE = PortSourceGPIOD
# v1.1
#MAX_1168_DRDY_PORT = GPIOB



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
