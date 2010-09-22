# for Tiny v1.1

ap.CFLAGS 	+= -DACTUATORS=\"servos_4015_MAT_hw.h\" -DSERVOS_4015_MAT
ap.srcs 	+= $(SRC_ARCH)/servos_4015_MAT_hw.c
ap.srcs 	+= $(SRC_FIXEDWING)/actuators.c

