# Vane sensors using generic PWM input
ap.CFLAGS += -DUSE_VANE_SENSOR
ap.srcs += $(SRC_CSC)/csc_vane.c

