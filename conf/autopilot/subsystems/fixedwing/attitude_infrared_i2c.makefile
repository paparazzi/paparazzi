# Hey Emacs, this is a -*- makefile -*-

# attitude via IR I2C sensors

$(TARGET).CFLAGS += -DUSE_INFRARED_I2C
$(TARGET).srcs += subsystems/sensors/infrared.c
$(TARGET).srcs += subsystems/sensors/infrared_i2c.c

sim.srcs += $(SRC_ARCH)/sim_ir.c
jsbsim.srcs += $(SRC_ARCH)/jsbsim_ir.c
