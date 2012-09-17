# Hey Emacs, this is a -*- makefile -*-
#
# lisa_m_2.0.makefile
#
# http://paparazzi.enac.fr/wiki/Lisa/M_v20
#

BOARD=lisa_m
BOARD_VERSION=2.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny

# -----------------------------------------------------------------------

ifndef FLASH_MODE
FLASH_MODE = DFU
#FLASH_MODE = JTAG
#FLASH_MODE = SERIAL
endif

ifndef NO_LUFTBOOT
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDSCRIPT = $(SRC_ARCH)/lisa_m_2.0_luftboot.ld
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
RADIO_CONTROL_LED  = 4
endif

ifndef BARO_LED
BARO_LED = none
endif

ifndef AHRS_ALIGNER_LED
AHRS_ALIGNER_LED = 2
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
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART1
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
GPS_PORT=UART3
endif
ifndef GPS_BAUD
GPS_BAUD=B38400
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
ADC_IR_TOP      = 3
ADC_IR_TOP_CHAN = 2
endif
ifndef ADC_IR_NB_SAMPLES
ADC_IR_NB_SAMPLES = 16
endif
