# for Tiny v2 or Twog v1

ap.CFLAGS += -DACTUATORS=\"servos_4017_hw.h\" -DSERVOS_4017
ap.srcs += $(SRC_FIXEDWING_ARCH)/servos_4017_hw.c $(SRC_FIXEDWING)/actuators.c

