#
# SITL Simulator
#

SIM_TYPE = BOOZ
BOOZ_PRIV_SIM = $(BOOZ_PRIV)/sim
#BOOZ_PRIV_ARCH = $(BOOZ_PRIV_SIM)


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy

sim.CFLAGS += -I $(BOOZ_PRIV) -I $(BOOZ_PRIV_SIM) -I../simulator -DFLOAT_T=float

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

sim.srcs   += $(BOOZ_PRIV_SIM)/booz2_unsimulated_peripherals.c
sim.srcs   += $(BOOZ_PRIV)/booz2_main_tmp.c

sim.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
#sim.CFLAGS += -DLED
sim.srcs += sys_time.c


sim.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport 
sim.srcs += $(BOOZ_PRIV)/booz2_telemetry.c \
            downlink.c \
            $(SRC_ARCH)/ivy_transport.c

#sim.CFLAGS += -DDATALINK=PPRZ -DPPRZ_UART=Uart1
#sim.srcs += $(BOOZ_PRIV)/booz2_datalink.c

sim.srcs   += $(BOOZ_PRIV)/booz2_commands.c

sim.CFLAGS += -DRADIO_CONTROL -DRADIO_CONTROL_TYPE=RC_FUTABA -DRC_LED=1
sim.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c

#sim.CFLAGS += -DACTUATORS=\"actuators_buss_twi_blmc_hw.h\" -DUSE_BUSS_TWI_BLMC
#sim.srcs += $(BOOZ_PRIV_SIM)/actuators_buss_twi_blmc_hw.c actuators.c
#sim.CFLAGS += -DUSE_I2C0 -DI2C0_SCLL=150 -DI2C0_SCLH=150 -DI2C0_VIC_SLOT=10
#sim.srcs += i2c.c $(SRC_ARCH)/i2c_hw.c






sim.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=2 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
sim.srcs += $(BOOZ_PRIV)/booz2_analog_baro.c

sim.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
sim.srcs += $(BOOZ_PRIV)/booz2_battery.c

sim.srcs += $(BOOZ_PRIV)/booz2_analog.c $(BOOZ_PRIV_SIM)/booz2_analog_hw.c




sim.srcs += $(BOOZ_PRIV)/booz2_gps.c

sim.srcs += $(BOOZ_PRIV)/booz2_autopilot.c

sim.CFLAGS += -DFILTER_ALIGNER_LED=3
sim.srcs += $(BOOZ_PRIV)/booz2_filter_aligner2.c
sim.srcs += $(BOOZ_PRIV)/booz2_filter_attitude_cmpl_euler.c
sim.srcs += $(BOOZ_PRIV)/booz_trig_int.c
sim.srcs += $(BOOZ_PRIV)/booz2_stabilization.c
sim.srcs += $(BOOZ_PRIV)/booz2_stabilization_rate.c
sim.srcs += $(BOOZ_PRIV)/booz2_stabilization_attitude.c

sim.srcs += $(BOOZ_PRIV)/booz2_guidance_h.c
sim.srcs += $(BOOZ_PRIV)/booz2_guidance_v.c
sim.srcs += $(BOOZ_PRIV)/booz2_ins.c
# vertical filter dummy complementary
#sim.CFLAGS += -DUSE_VFD
#  vertical filter float version
sim.srcs += $(BOOZ_PRIV)/booz2_vf_float.c
sim.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)" -DFLOAT_T=float



sim.srcs += $(BOOZ_PRIV)/booz2_navigation.c

sim.srcs += $(BOOZ_PRIV)/booz2_fms.c
sim.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_TEST_SIGNAL
sim.srcs += $(BOOZ_PRIV)/booz2_fms_test_signal.c