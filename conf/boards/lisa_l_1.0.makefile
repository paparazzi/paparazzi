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

	ARCHI  = omap
	$(TARGET).LDFLAGS += -levent -lm

# -----------------------------------------------------------------------
else

	ARCH=stm32
	ARCHI=stm32

	$(TARGET).ARCHDIR = $(ARCHI)
# not needed?
#	$(TARGET).ARCH = arm7tdmi
	$(TARGET).TARGET = $(TARGET)
	$(TARGET).TARGETDIR = $(TARGET)

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