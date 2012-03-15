# Hey Emacs, this is a -*- makefile -*-

#
# Swap GPS UART with spektrum UART
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART1

ifndef GPS_PORT
GPS_PORT=UART3
endif

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_1.0.makefile
