#
# SITL Simulator
#

SIM_TYPE = JSBSIM

JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_BOOZ=booz
SRC_BOOZ_SIM = $(SRC_BOOZ)/arch/sim

NPSDIR = $(SIMDIR)/nps


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy
sim.CFLAGS  += -I$(NPSDIR) -I/usr/local/include -I$(JSBSIM_INC)
sim.LDFLAGS += -L$(JSBSIM_LIB) -lJSBSim

sim.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_SIM) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator/nps

sim.srcs = $(NPSDIR)/nps_main.c                      \
	   $(NPSDIR)/nps_fdm_jsbsim.c                \
	   $(NPSDIR)/nps_random.c                    \
	   $(NPSDIR)/nps_sensors.c                   \
	   $(NPSDIR)/nps_sensor_gyro.c               \
	   $(NPSDIR)/nps_sensor_accel.c              \
	   $(NPSDIR)/nps_sensor_mag.c                \
	   $(NPSDIR)/nps_sensor_baro.c               \
	   $(NPSDIR)/nps_sensor_gps.c                \
	   $(NPSDIR)/nps_radio_control.c             \
	   $(NPSDIR)/nps_autopilot_booz.c            \
	   $(NPSDIR)/nps_ivy.c                       \
	   $(NPSDIR)/nps_flightgear.c                \


sim.srcs += math/pprz_trig_int.c             \
            math/pprz_geodetic_float.c       \
            math/pprz_geodetic_double.c      \



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


#
#
#


sim.CFLAGS += -DBOOZ2_ANALOG_BARO_LED=2 -DBOOZ2_ANALOG_BARO_PERIOD='SYS_TICS_OF_SEC((1./100.))'
sim.srcs += $(SRC_BOOZ)/booz2_analog_baro.c

sim.CFLAGS += -DBOOZ2_ANALOG_BATTERY_PERIOD='SYS_TICS_OF_SEC((1./10.))'
sim.srcs += $(SRC_BOOZ)/booz2_battery.c

sim.srcs += $(SRC_BOOZ)/booz2_analog.c $(SRC_BOOZ_SIM)/booz2_analog_hw.c




sim.srcs += $(SRC_BOOZ)/booz2_autopilot.c

sim.CFLAGS += -DAHRS_ALIGNER_LED=3
sim.srcs += $(SRC_BOOZ)/ahrs/booz_ahrs_aligner.c
sim.srcs += $(SRC_BOOZ)/ahrs/booz2_filter_attitude_cmpl_euler.c

sim.srcs += $(SRC_BOOZ)/booz_stabilization.c
sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_rate.c

#sim.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
#sim.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/booz_stabilization_attitude_int.h\"
#sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/booz_stabilization_attitude_ref_euler_int.h\"
#sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_ref_euler_int.c
#sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_euler_int.c


#sim.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_FLOAT
#sim.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/booz_stabilization_attitude_float.h\"
#sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/booz_stabilization_attitude_ref_euler_float.h\"
#sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_ref_euler_float.c
#sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_euler_float.c

sim.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_FLOAT
sim.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/booz_stabilization_attitude_float.h\"
sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/booz_stabilization_attitude_ref_quat_float.h\"
sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_ref_quat_float.c
sim.srcs += $(SRC_BOOZ)/stabilization/booz_stabilization_attitude_quat_float.c




sim.srcs += $(SRC_BOOZ)/guidance/booz2_guidance_h.c
sim.srcs += $(SRC_BOOZ)/guidance/booz2_guidance_v.c
sim.srcs += math/pprz_geodetic_int.c
sim.srcs += $(SRC_BOOZ)/booz2_ins.c

#  vertical filter float version
sim.srcs += $(SRC_BOOZ)/ins/booz2_vf_float.c
sim.CFLAGS += -DUSE_VFF -DDT_VFILTER="(1./512.)"
sim.srcs += $(SRC_BOOZ)/ins/booz2_hf_float.c



sim.srcs += $(SRC_BOOZ)/booz2_navigation.c


