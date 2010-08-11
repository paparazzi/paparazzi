# for Tiny v1.1

ap.CFLAGS += -DACTUATORS=\"servos_4015_MAT_hw.h\" -DSERVOS_4015_MAT
ap.srcs += $(SRC_FIXEDWING_ARCH)/servos_4015_MAT_hw.c $(SRC_FIXEDWING)/actuators.c

