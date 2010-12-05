#generic i2c driver
ap.srcs += mcu_periph/i2c.c
ap.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
sim.srcs += mcu_periph/i2c.c
sim.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c
