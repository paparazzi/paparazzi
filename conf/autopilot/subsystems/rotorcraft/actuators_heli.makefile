
ap.srcs += $(SRC_BOOZ)/actuators/booz_actuators_heli.c
ap.srcs += $(SRC_BOOZ_ARCH)/actuators/booz_actuators_pwm_arch.c

# fixme : this is needed by baro and usualy added by actuators_mkk or actuators_asctec
ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c