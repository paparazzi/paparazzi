# attitude via IR sensors and standard assignment for tiny and twog

ap.CFLAGS += -DADC_CHANNEL_IR1=$(ADC_IR1) -DUSE_$(ADC_IR1)
ap.CFLAGS += -DADC_CHANNEL_IR2=$(ADC_IR2) -DUSE_$(ADC_IR2)
ap.CFLAGS += -DADC_CHANNEL_IR_TOP=$(ADC_IR_TOP) -DUSE_$(ADC_IR_TOP)
ap.CFLAGS += -DADC_CHANNEL_IR_NB_SAMPLES=$(ADC_IR_NB_SAMPLES)

ap.CFLAGS += -DINFRARED
ap.srcs += $(SRC_FIXEDWING)/infrared.c

