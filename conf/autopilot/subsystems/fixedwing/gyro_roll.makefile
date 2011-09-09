# Hey Emacs, this is a -*- makefile -*-

# roll stabilization with gyro

#
# default values for tiny and twog are:
# ADC_GYRO_ROLL    = ADC_3
# ADC_GYRO_NB_SAMPLES = 16
#
# to change just redefine these before including this file
#

ap.CFLAGS += -DADC_CHANNEL_GYRO_ROLL=$(ADC_GYRO_ROLL) -DUSE_$(ADC_GYRO_ROLL)
ap.CFLAGS += -DADC_CHANNEL_GYRO_NB_SAMPLES=$(ADC_GYRO_NB_SAMPLES)

ap.CFLAGS += -DUSE_GYRO -DADXRS150
ap.srcs += $(SRC_FIXEDWING)/gyro.c
