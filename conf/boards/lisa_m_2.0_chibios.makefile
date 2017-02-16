# Hey Emacs, this is a -*- makefile -*-
#
# Lisa_m_2.0_chibios.makefile
#
#

BOARD=lisa_m
BOARD_VERSION=2.0

#
# default PPM input is on PA01 (SERVO6)
#
RADIO_CONTROL_PPM_PIN ?= PA01
ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_10 PA10 UART1_RX))
  PPM_CONFIG=1
else ifeq ($(RADIO_CONTROL_PPM_PIN),$(filter $(RADIO_CONTROL_PPM_PIN),PA_01 PA01 PA1 SERVO6))
  PPM_CONFIG=2
else
$(error Unknown RADIO_CONTROL_PPM_PIN, configure it to either PA01 or PA10)
endif

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_defaults.makefile
include $(PAPARAZZI_SRC)/conf/boards/lisa_m_common_chibios.makefile
