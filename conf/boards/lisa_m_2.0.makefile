# Hey Emacs, this is a -*- makefile -*-

include $(PAPARAZZI_SRC)/conf/boards/lisa_m_1.0.makefile

#
# Swap GPS UART with spektrum UART
#
RADIO_CONTROL_SPEKTRUM_PRIMARY_PORT   = UART1
GPS_PORT=UART3

#
# Disable aligner led for now.
#
AHRS_ALIGNER_LED = none
