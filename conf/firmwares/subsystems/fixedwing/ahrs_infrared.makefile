# Hey Emacs, this is a -*- makefile -*-

# attitude estimation for fixedwings using infrared sensors


# usage of this ahrs subsystem implies USE_INFRARED
$(TARGET).CFLAGS += -DUSE_INFRARED

$(TARGET).CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_infrared.h\"
$(TARGET).CFLAGS += -DUSE_AHRS

$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs.c
$(TARGET).srcs   += $(SRC_SUBSYSTEMS)/ahrs/ahrs_infrared.c
