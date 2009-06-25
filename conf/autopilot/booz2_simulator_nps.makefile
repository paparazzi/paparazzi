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
	   $(SIMDIR)/nps_autopilot.c                 \
	   $(SIMDIR)/nps_ivy.c                       \
	   $(SIMDIR)/nps_flightgear.c                \


sim.srcs += $(SRC_BOOZ)/booz_trig_int.c \
		pprz_geodetic_float.c

