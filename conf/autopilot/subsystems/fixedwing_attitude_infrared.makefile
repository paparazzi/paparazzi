# attitude via IR sensors and standard assignment for tiny and twog

ap.CFLAGS += -DUSE_$(ADC_CHANNEL_IR1) -DUSE_$(ADC_CHANNEL_IR2) -DUSE_$(ADC_CHANNEL_IR_TOP)

ap.CFLAGS += -DINFRARED
ap.srcs += $(SRC_FIXEDWING)/infrared.c

