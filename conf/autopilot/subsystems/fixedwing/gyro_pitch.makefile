# Hey Emacs, this is a -*- makefile -*-

# additional pitch stabilization with gyro
#
# this assumes you are already using a roll gyro
#
# default values for tiny and twog are:
# ADC_GYRO_PITCH    = ADC_4
#
# to change just redefine these before including this file
#

ap.CFLAGS += -DADC_CHANNEL_GYRO_PITCH=$(ADC_GYRO_PITCH) -DUSE_$(ADC_GYRO_PITCH)

