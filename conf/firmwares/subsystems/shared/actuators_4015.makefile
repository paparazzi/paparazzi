# for Tiny v1.1

$(TARGET).CFLAGS 	+= -DACTUATORS -DSERVOS_4015_MAT
$(TARGET).srcs 	+= $(SRC_ARCH)/subsystems/actuators/servos_4015_MAT_hw.c
