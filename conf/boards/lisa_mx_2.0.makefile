# Hey Emacs, this is a -*- makefile -*-
#
# lisa_mx_2.0.makefile
#
# http://wiki.paparazziuav.org/wiki/Lisa/M_v20
#

BOARD=lisa_mx
BOARD_VERSION=2.0
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

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_defaults.makefile
