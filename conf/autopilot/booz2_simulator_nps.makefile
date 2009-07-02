#
# SITL Simulator
#

SIM_TYPE = JSBSIM

JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_BOOZ=booz
SRC_BOOZ_SIM = $(SRC_BOOZ)/sim


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy
sim.CFLAGS  += -I$(SIMDIR) -I/usr/local/include -I$(JSBSIM_INC)
sim.LDFLAGS += -L$(JSBSIM_LIB) -lJSBSim

sim.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_SIM) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator/nps

sim.srcs = $(SIMDIR)/nps_main.c                      \
	   $(SIMDIR)/nps_fdm_jsbsim.c                \
	   $(SIMDIR)/nps_random.c                    \
	   $(SIMDIR)/booz_r250.c                     \
	   $(SIMDIR)/booz_randlcg.c                  \
	   $(SIMDIR)/nps_sensors.c                   \
	   $(SIMDIR)/nps_sensor_gyro.c               \
	   $(SIMDIR)/nps_sensor_accel.c              \
	   $(SIMDIR)/nps_sensor_mag.c                \
	   $(SIMDIR)/nps_sensor_baro.c               \
	   $(SIMDIR)/nps_sensor_gps.c                \
	   $(SIMDIR)/nps_radio_control.c             \
	   $(SIMDIR)/nps_autopilot_booz.c            \
	   $(SIMDIR)/nps_ivy.c                       \
	   $(SIMDIR)/nps_flightgear.c                \


sim.srcs += $(SRC_BOOZ)/booz_trig_int.c \
            pprz_geodetic_float.c       \
            pprz_geodetic_double.c      \



sim.CFLAGS += -DCONFIG=\"booz2_board_v1_0.h\"

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




sim.srcs += $(SRC_BOOZ)/booz2_gps.c

sim.srcs += $(SRC_BOOZ)/booz2_autopilot.c

sim.CFLAGS += -DAHRS_ALIGNER_LED=3
sim.srcs += $(SRC_BOOZ)/booz_ahrs_aligner.c
sim.srcs += $(SRC_BOOZ)/booz2_filter_attitude_cmpl_euler.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization_rate.c
sim.srcs += $(SRC_BOOZ)/booz2_stabilization_attitude.c

sim.srcs += $(SRC_BOOZ)/booz2_guidance_h.c
sim.srcs += $(SRC_BOOZ)/booz2_guidance_v.c
sim.srcs += pprz_geodetic_int.c
sim.srcs += $(SRC_BOOZ)/booz2_ins.c
#  vertical filter float version
sim.srcs += $(SRC_BOOZ)/booz2_vf_float.c
sim.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)"
sim.srcs += $(SRC_BOOZ)/booz2_hf_float.c



sim.srcs += $(SRC_BOOZ)/booz2_navigation.c

sim.srcs += $(SRC_BOOZ)/booz2_fms.c
sim.CFLAGS += -DBOOZ2_FMS_TYPE=BOOZ2_FMS_TYPE_TEST_SIGNAL
sim.srcs += $(SRC_BOOZ)/booz2_fms_test_signal.c


