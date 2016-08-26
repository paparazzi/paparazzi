# Hey Emacs, this is a -*- makefile -*-

# fixed wing control loops with adaptive horizontal control


$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_adaptive.c $(SRC_FIRMWARE)/guidance/guidance_v.c

$(TARGET).CFLAGS += -DCTRL_TYPE_H=\"firmwares/fixedwing/guidance/guidance_v.h\"

