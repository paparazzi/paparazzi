# Standard setup for roll stabilization with gyro on a tiny or twog

ap.CFLAGS 	+= -DUSE_$(ADC_CHANNEL_GYRO_ROLL)
ap.CFLAGS 	+= -DGYRO -DADXRS150
ap.srcs 	+= $(SRC_FIXEDWING)/gyro.c
