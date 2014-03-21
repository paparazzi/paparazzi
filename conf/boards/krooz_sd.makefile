# Hey Emacs, this is a -*- makefile -*-
#
# krooz_sd.makefile
#
#
#

BOARD=krooz
BOARD_VERSION=sd
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
ARCH_DIR=stm32
SRC_ARCH=arch/$(ARCH_DIR)
$(TARGET).ARCHDIR = $(ARCH)
# not needed?
$(TARGET).OOCD_INTERFACE=flossjtag
#$(TARGET).OOCD_INTERFACE=jtagkey-tiny
$(TARGET).LDSCRIPT=$(SRC_ARCH)/krooz.ld

# -----------------------------------------------------------------------

# default flash mode is via usb dfu bootloader (luftboot)
# other possibilities: DFU-UTIL, JTAG, SWD, STLINK, SERIAL
FLASH_MODE ?= DFU

DFU_ADDR = 0x8004000
DFU_PRODUCT = any

#
#
# some default values shared between different firmwares
#
#

#
# default LED configuration
#
RADIO_CONTROL_LED ?= none
BARO_LED ?= none
AHRS_ALIGNER_LED ?= 2
GPS_LED ?= none
SYS_TIME_LED ?= 1

#
# default uart configuration
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT ?= UART1

MODEM_PORT ?= UART5
MODEM_BAUD ?= B57600

GPS_PORT ?= UART3
GPS_BAUD ?= B38400
