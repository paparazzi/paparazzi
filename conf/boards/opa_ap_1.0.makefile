# Hey Emacs, this is a -*- makefile -*-
#
# Oversized Paparazzi (AP) Board
#
# http://wiki.paparazziuav.org/wiki/OPA
#

BOARD=opa_ap
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-mx.ld

# -----------------------------------------------------------------------

#
# default flash mode is via SWD (JTAG)
#
FLASH_MODE ?= SWD

#
# default LED configuration
#
SYS_TIME_LED       ?= 1
RADIO_CONTROL_LED  ?= none
BARO_LED           ?= none
AHRS_ALIGNER_LED   ?= none
GPS_LED            ?= none
LOGGER_LED         ?= none

#
# default uart configuration
#
MODEM_PORT ?= UART2
MODEM_BAUD ?= B57600

GPS_PORT ?= UART1
GPS_BAUD ?= B57600

INTERMCU_PORT ?= UART3
INTERMCU_BAUD ?= B460800

#
# default IMU configuration
#
IMU_MPU_SPI_DEV ?= spi2
IMU_MPU_SPI_SLAVE_IDX ?= SPI_SLAVE1

#
# BARO configuration
#

# See baro_board.makefile


#
# Remote Magneto
#

MAG_PITOT_PORT ?= UART5
MAG_PITOT_BAUD ?= 250000

#
# default SPI logger configuration
#
SDLOGGER_DIRECT_SPI ?= spi1
SDLOGGER_DIRECT_SPI_SLAVE ?= SPI_SLAVE0
HS_LOG_SPI_DEV ?= spi1
HS_LOG_SPI_SLAVE_IDX ?= SPI_SLAVE0

#
# default Current Sensor configuration
#
ADC_CURRENT_SENSOR ?= ADC_2
