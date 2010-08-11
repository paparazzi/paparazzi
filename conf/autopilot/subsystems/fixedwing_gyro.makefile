# Standard setup for roll stabilization with gyro on a tiny or twog

ap.CFLAGS += -DADC_CHANNEL_GYRO_ROLL=$(ADC_GYRO_ROLL) -DUSE_$(ADC_GYRO_ROLL)
ap.CFLAGS += -DADC_CHANNEL_GYRO_NB_SAMPLES=$(ADC_GYRO_NB_SAMPLES)

ap.CFLAGS += -DGYRO -DADXRS150
ap.srcs += $(SRC_FIXEDWING)/gyro.c
