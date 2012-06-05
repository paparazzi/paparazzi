# for Tiny v1.1

$(TARGET).CFLAGS 	+= -DACTUATORS=\"servos_4015_MAT_hw.h\" -DSERVOS_4015_MAT
$(TARGET).srcs 	+= $(SRC_ARCH)/servos_4015_MAT_hw.c actuators.c
