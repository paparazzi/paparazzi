# generic ADC

#
# default values for tiny and twog are:
# ADC_GENERIC_NB_SAMPLES = 16
#
# to change just redefine before including this file
#
# you have to set the params of the ADCs to use, eg.
# <subsystem name"adc" type="generic">
#   <param name="ADC_GENERIC1" value="ADC_3"/>
#   <param name="ADC_GENERIC2" value="ADC_4"/>
# </subsystem>
#
# if you only set one parameter only that one will be used

ifdef ADC_GENERIC1
  ap.CFLAGS += -DUSE_ADC_GENERIC -DADC_CHANNEL_GENERIC_NB_SAMPLES=$(ADC_GENERIC_NB_SAMPLES)
  ap.CFLAGS += -DADC_CHANNEL_GENERIC1=$(ADC_GENERIC1) -DUSE_$(ADC_GENERIC1)
  ifdef ADC_GENERIC2
    ap.CFLAGS += -DADC_CHANNEL_GENERIC2=$(ADC_GENERIC2) -DUSE_$(ADC_GENERIC2)
  endif
  ap.srcs += $(SRC_FIXEDWING)/adc_generic.c
else
  ifdef ADC_GENERIC2
    ap.CFLAGS += -DUSE_ADC_GENERIC -DADC_CHANNEL_GENERIC_NB_SAMPLES=$(ADC_GENERIC_NB_SAMPLES)
    ap.CFLAGS += -DADC_CHANNEL_GENERIC2=$(ADC_GENERIC1) -DUSE_$(ADC_GENERIC2)
    ap.srcs += $(SRC_FIXEDWING)/adc_generic.c
  endif
endif
