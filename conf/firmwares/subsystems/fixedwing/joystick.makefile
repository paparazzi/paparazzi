# Hey Emacs, this is a -*- makefile -*-

# joystick for fixedwings

$(TARGET).srcs += joystick.c
$(TARGET).CFLAGS += -DUSE_JOYSTICK

