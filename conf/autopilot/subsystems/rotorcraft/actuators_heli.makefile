
# add actuatos arch to include directories
ap.CFLAGS += -I$(SRC_FIRMWARE)/actuators/arch/$(ARCH)

ap.srcs += $(SRC_FIRMWARE)/actuators/actuators_heli.c
ap.srcs += $(SRC_FIRMWARE)/actuators/arch/$(ARCH)/actuators_pwm_arch.c

# fixme : this is needed by baro and usualy added by actuators_mkk or actuators_asctec
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
