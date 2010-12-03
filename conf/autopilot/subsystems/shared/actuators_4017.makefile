# for Tiny v2 or Twog v1

$(TARGET).CFLAGS += -DACTUATORS=\"servos_4017_hw.h\" -DSERVOS_4017
$(TARGET).srcs += $(SRC_ARCH)/servos_4017_hw.c actuators.c
