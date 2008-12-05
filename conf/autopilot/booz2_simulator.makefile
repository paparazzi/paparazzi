#
# SITL Simulator
#

SIM_TYPE = BOOZ
SIM_PRIV_ARCH = $(BOOZ_PRIV)/sim
#BOOZ_PRIV_ARCH = $(SIM_PRIV_ARCH)


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy

sim.CFLAGS += -I $(BOOZ_PRIV) -I $(SIM_PRIV_ARCH) -I../simulator -DFLOAT_T=float
sim.CFLAGS += -DBSM_PARAMS=\"booz_sensors_model_params_booz2.h\"

sim.srcs = $(SIMDIR)/booz2_sim_main.c                \
	   $(SIMDIR)/booz_flight_model.c             \
           $(SIMDIR)/booz_flight_model_utils.c       \
           $(SIMDIR)/booz_sensors_model.c            \
	   $(SIMDIR)/booz_sensors_model_utils.c      \
           $(SIMDIR)/booz_sensors_model_accel.c      \
           $(SIMDIR)/booz_sensors_model_gyro.c       \
           $(SIMDIR)/booz_sensors_model_mag.c        \
           $(SIMDIR)/booz_sensors_model_rangemeter.c \
           $(SIMDIR)/booz_sensors_model_baro.c       \
           $(SIMDIR)/booz_sensors_model_gps.c        \
           $(SIMDIR)/booz_wind_model.c               \

sim.CFLAGS += -DCONFIG=\"booz2_board_v1_0.h\"

sim.srcs   += $(BOOZ_PRIV)/booz2_main.c


sim.srcs   += commands.c

sim.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=1
sim.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
sim.srcs += $(SIM_PRIV_ARCH)/actuators_buss_twi_blmc_hw.c actuators.c
sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c




sim.CFLAGS += -DBOOZ2_IMU_TYPE=IMU_B2
sim.CFLAGS += -DUSE_I2C1  -DI2C1_SCLL=150 -DI2C1_SCLH=150 -DI2C1_VIC_SLOT=11 -DI2C1_BUF_LEN=16
#ap.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c
sim.CFLAGS += -DUSE_AMI601
sim.srcs += AMI601.c


sim.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_TEST_SIGNAL
sim.srcs += $(BOOZ_PRIV)/booz2_fms_test_signal.c