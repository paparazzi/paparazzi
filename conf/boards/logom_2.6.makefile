#
# logom_2.6.makefile
#
# http://www.sparkfun.com/products/10216
#

ARCH=lpc21

BOARD=logom
BOARD_VERSION=2.6

BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ifndef FLASH_MODE
FLASH_MODE = IAP
endif


LPC21ISP_BAUD = 38400
LPC21ISP_XTAL = 12000


ADC_GENERIC_NB_SAMPLES = 16

# All targets on the board run on the same processor achitecture
$(TARGET).ARCHDIR = $(ARCH)

