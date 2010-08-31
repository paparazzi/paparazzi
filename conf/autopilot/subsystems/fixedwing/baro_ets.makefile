# EagleTree sensors (altimeter and airspeed)
ap.CFLAGS += -DUSE_BARO_ETS -DUSE_I2C0
ap.srcs += baro_ets.c i2c.c $(SRC_ARCH)/i2c_hw.c

sim.CFLAGS += -DUSE_BARO_ETS -DUSE_I2C0
sim.srcs += baro_ets.c i2c.c $(SRC_ARCH)/i2c_hw.c
