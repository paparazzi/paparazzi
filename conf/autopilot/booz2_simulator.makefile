#
# SITL Simulator
#

SIM_TYPE = BOOZ
SRC_BOOZ=booz
SRC_BOOZ_SIM = $(SRC_BOOZ)/sim
#BOOZ_PRIV_ARCH = $(BOOZ_PRIV_SIM)


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy

#sim.CFLAGS  += -DBYPASS_AHRS
#sim.CFLAGS  += -DBYPASS_INS
#sim.CFLAGS  += -DGPS_PERFECT
sim.CFLAGS  += -DUSE_HFF

sim.CFLAGS  += -DINIT_WIND_X=-3.0
sim.CFLAGS  += -DINIT_WIND_Y=-0.0
sim.CFLAGS  += -DINIT_WIND_Z=-0.0


sim.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_SIM) -I../simulator -DFLOAT_T=float

sim.srcs = $(SIMDIR)/booz2_sim_main.c                \
	   $(SIMDIR)/booz_flight_model.c             \
           $(SIMDIR)/booz_flight_model_utils.c       \
           $(SIMDIR)/booz_sensors_model.c            \
	   $(SIMDIR)/booz_sensors_model_utils.c      \
	   pprz_geodetic_double.c	             \
	   $(SIMDIR)/booz_r250.c      		     \
	   $(SIMDIR)/booz_randlcg.c	             \
	   $(SIMDIR)/booz_sensors_model_accel.c      \
           $(SIMDIR)/booz_sensors_model_gyro.c       \
           $(SIMDIR)/booz_sensors_model_mag.c        \
           $(SIMDIR)/booz_sensors_model_rangemeter.c \
           $(SIMDIR)/booz_sensors_model_baro.c       \
           $(SIMDIR)/booz_sensors_model_gps.c        \
           $(SIMDIR)/booz_wind_model.c               \

sim.CFLAGS += -DBOARD_CONFIG=\"booz2_board_v1_0.h\"

sim.srcs   += $(SRC_BOOZ_SIM)/booz2_unsimulated_peripherals.c
sim.srcs   += $(SRC_BOOZ)/booz2_main.c

sim.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
#sim.CFLAGS += -DLED
sim.srcs += sys_time.c


sim.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport 
sim.srcs += $(SRC_BOOZ)/booz2_telemetry.c \
            downlink.c \
            $(SRC_ARCH)/ivy_transport.c

sim.srcs   += $(SRC_BOOZ)/booz2_commands.c

sim.CFLAGS += -DRADIO_CONTROL -DRC_LED=1
sim.srcs += radio_control.c $(SRC_ARCH)/ppm_hw.c


sim.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=2 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
sim.srcs += $(SRC_BOOZ)/booz2_analog_baro.c

sim.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
sim.srcs += $(SRC_BOOZ)/booz2_battery.c

sim.srcs += $(SRC_BOOZ)/booz2_analog.c $(SRC_BOOZ_SIM)/booz2_analog_hw.c





sim.srcs += $(SRC_BOOZ)/booz2_autopilot.c

sim.CFLAGS += -DAHRS_ALIGNER_LED=3
sim.srcs += $(SRC_BOOZ)/booz_ahrs_aligner.c
sim.srcs += $(SRC_BOOZ)/booz2_filter_attitude_cmpl_euler.c
sim.srcs += $(SRC_BOOZ)/booz_trig_int.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization_rate.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization_attitude.c

sim.srcs += $(SRC_BOOZ)/booz2_guidance_h.c
sim.srcs += $(SRC_BOOZ)/booz2_guidance_v.c
sim.srcs += pprz_geodetic_int.c pprz_geodetic_float.c
sim.srcs += $(SRC_BOOZ)/booz2_ins.c
#  vertical filter float version
sim.srcs += $(SRC_BOOZ)/booz2_vf_float.c
sim.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)"
sim.srcs += $(SRC_BOOZ)/booz2_hf_float.c



sim.srcs += $(SRC_BOOZ)/booz2_navigation.c

sim.srcs += $(SRC_BOOZ)/booz2_fms.c
sim.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_TEST_SIGNAL
sim.srcs += $(SRC_BOOZ)/booz2_fms_test_signal.c