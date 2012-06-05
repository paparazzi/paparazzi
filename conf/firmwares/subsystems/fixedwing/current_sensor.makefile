# Hey Emacs, this is a -*- makefile -*-

# Activate current sensor

#
# MilliAmpereOfAdc needs to be defined in the airframe file
#

ap.CFLAGS += -DADC_CHANNEL_CURRENT=$(ADC_CURRENT_SENSOR) -DUSE_$(ADC_CURRENT_SENSOR)

