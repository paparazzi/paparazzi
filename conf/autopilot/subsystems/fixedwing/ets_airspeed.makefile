# EagleTree sensors (altimeter and airspeed)
ap.CFLAGS += -DUSE_AIRSPEED_ETS -DUSE_AIRSPEED -DUSE_BARO_ETS -DUSE_I2C0
ap.srcs += airspeed.c airspeed_ets.c baro_ets.c i2c.c $(SRC_ARCH)/i2c_hw.c

sim.CFLAGS += -DUSE_AIRSPEED_ETS -DUSE_AIRSPEED -DUSE_BARO_ETS -DUSE_I2C0
sim.srcs += airspeed.c airspeed_ets.c baro_ets.c i2c.c $(SRC_ARCH)/i2c_hw.c
