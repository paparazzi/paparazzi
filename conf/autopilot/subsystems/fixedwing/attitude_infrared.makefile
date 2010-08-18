# attitude via IR sensors

#
# default values for tiny and twog are:
# ADC_IR1    = ADC_1
# ADC_IR2    = ADC_2
# ADC_IR_TOP = ADC_0
# ADC_IR_NB_SAMPLES = 16
#
# to change just redefine these before including this file
#

ap.CFLAGS += -DADC_CHANNEL_IR1=$(ADC_IR1) -DUSE_$(ADC_IR1)
ap.CFLAGS += -DADC_CHANNEL_IR2=$(ADC_IR2) -DUSE_$(ADC_IR2)
ap.CFLAGS += -DADC_CHANNEL_IR_TOP=$(ADC_IR_TOP) -DUSE_$(ADC_IR_TOP)
ap.CFLAGS += -DADC_CHANNEL_IR_NB_SAMPLES=$(ADC_IR_NB_SAMPLES)

ap.CFLAGS += -DINFRARED
ap.srcs += $(SRC_FIXEDWING)/infrared.c

