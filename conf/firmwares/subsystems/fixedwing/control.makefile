# Hey Emacs, this is a -*- makefile -*-

# Standard fixed wing control loops


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude.c $(SRC_FIRMWARE)/guidance/guidance_v.c

$(TARGET).CFLAGS += -DCTRL_TYPE_H=\"firmwares/fixedwing/guidance/guidance_v.h\"

