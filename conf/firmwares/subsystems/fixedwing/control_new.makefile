# Hey Emacs, this is a -*- makefile -*-

# new fixed wing control loops with merged auto pitch and auto throttle, adaptive horizontal control

$(TARGET).srcs += $(SRC_FIRMWARE)/stabilization/stabilization_adaptive.c $(SRC_FIRMWARE)/guidance/guidance_v_n.c

$(TARGET).CFLAGS += -DCTRL_TYPE_H=\"firmwares/fixedwing/guidance/guidance_v.h\"


