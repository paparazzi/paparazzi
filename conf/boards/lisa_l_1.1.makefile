# Hey Emacs, this is a -*- makefile -*-
#
# lisa_l_1.1.makefile
#
# http://paparazzi.enac.fr/wiki/User/LisaL
#


# we are actually still using the Lisa/L 1.0 header file

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

#
# default LED configuration
#
ifndef RADIO_CONTROL_LED
RADIO_CONTROL_LED  = 5
endif

ifndef BARO_LED
BARO_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = 7
endif

ifndef GPS_LED
GPS_LED = 3
endif

ifndef SYS_TIME_LED
SYS_TIME_LED = 1
endif


#
# default uart configuration
#
ifndef RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART3
endif
ifndef RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT
RADIO_CONTROL_SPEKTRUM_SECONDARY_PORT = UART5
endif

ifndef MODEM_PORT
MODEM_PORT=UART2
endif
ifndef MODEM_BAUD
MODEM_BAUD=B57600
endif

ifndef GPS_PORT
GPS_PORT=UART1
endif
ifndef GPS_BAUD
GPS_BAUD=B38400
endif

#
# this is the DRDY pin of a max1168 on a booz IMU
#
# v 1.1
#
MAX_1168_DRDY_PORT = _GPIOB
MAX_1168_DRDY_PORT_SOURCE = PortSourceGPIOB
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
