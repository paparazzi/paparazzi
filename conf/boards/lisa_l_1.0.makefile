#
# lisa_l_1.0.makefile
#
# http://paparazzi.enac.fr/wiki/User/LisaL
#

BOARD=lisa_l
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

# -----------------------------------------------------------------------
ifeq ($(BOARD_PROCESSOR),'omap')

	ARCH  = omap
	$(TARGET).LDFLAGS += -levent -lm

# -----------------------------------------------------------------------
else

	ARCH=stm32

	$(TARGET).ARCHDIR = $(ARCH)
# not needed?

endif
# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = JTAG
endif

#
#
# some default values shared between different firmwares
#
#

SYS_TIME_LED       = 1

RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART3
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT = UART5
RADIO_CONTROL_LED  = 5

MODEM_PORT=UART2
MODEM_BAUD=B57600

AHRS_ALIGNER_LED = 7

GPS_PORT=UART1
GPS_BAUD=B38400
GPS_LED = 3

#
# this is the DRDY pin of a max1168 on a booz IMU
#
# v 1.0
#
MAX_1168_DRDY_PORT = _GPIOD
MAX_1168_DRDY_PORT_SOURCE = PortSourceGPIOD
# v1.1
#MAX_1168_DRDY_PORT = GPIOB


#
# definitions below were added to get fixedwing firmware to build
#
# modem is usualy connected there
ifndef MODEM_UART_NR
MODEM_UART_NR = 2
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
ifndef GPS_UART_NR
GPS_UART_NR	= 1
endif
