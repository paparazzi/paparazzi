# for Tiny v2 or Twog v1

$(TARGET).CFLAGS += -DACTUATORS -DSERVOS_4017
$(TARGET).srcs += $(SRC_ARCH)/subsystems/actuators/servos_4017_hw.c
