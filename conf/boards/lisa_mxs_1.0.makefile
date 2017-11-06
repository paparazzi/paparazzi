# Hey Emacs, this is a -*- makefile -*-
#
# lisa_mxs_1.0.makefile
#
# https://wiki.paparazziuav.org/wiki/Lisa/MXS_v1.0
#

BOARD=lisa_mxs
BOARD_VERSION=1.0
BOARD_CFG=\"boards/$(BOARD)_$(BOARD_VERSION).h\"

ARCH=stm32
ARCH_L=f4
HARD_FLOAT=yes
$(TARGET).ARCHDIR = $(ARCH)
$(TARGET).LDSCRIPT=$(SRC_ARCH)/lisa-mx.ld

# -----------------------------------------------------------------------

HAS_LUFTBOOT ?= 0
ifeq (,$(findstring $(HAS_LUFTBOOT),0 FALSE))
$(TARGET).CFLAGS+=-DLUFTBOOT
$(TARGET).LDFLAGS+=-Wl,-Ttext=0x8004000
DFU_ADDR = 0x8004000
DFU_PRODUCT = Lisa/Lia
endif

FLASH_MODE ?= SWD_NOPWR

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_defaults.makefile
