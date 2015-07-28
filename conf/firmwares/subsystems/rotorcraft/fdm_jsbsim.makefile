# Hey Emacs, this is a -*- makefile -*-

#
# SITL Simulator
#

JSBSIM_ROOT ?= /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_FIRMWARE=firmwares/rotorcraft

SRC_BOARD=boards/$(BOARD)

nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.CFLAGS  += $(shell pkg-config glib-2.0 --cflags)
nps.LDFLAGS += $(shell pkg-config glib-2.0 --libs) -lm -lglibivy $(shell pcre-config --libs) -lgsl -lgslcblas
nps.CFLAGS  += -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I$(PAPARAZZI_SRC)/sw/simulator -I$(PAPARAZZI_SRC)/sw/simulator/nps -I$(PAPARAZZI_HOME)/conf/simulator/nps
nps.LDFLAGS += $(shell sdl-config --libs)

# use the paparazzi-jsbsim package if it is installed, otherwise look for JSBsim under /opt/jsbsim
JSBSIM_PKG ?= $(shell pkg-config JSBSim --exists && echo 'yes')
ifeq ($(JSBSIM_PKG), yes)
	nps.CFLAGS  += $(shell pkg-config JSBSim --cflags)
	nps.LDFLAGS += $(shell pkg-config JSBSim --libs)
else
	JSBSIM_PKG = no
	nps.CFLAGS  += -I$(JSBSIM_INC)
	nps.LDFLAGS += -L$(JSBSIM_LIB) -lJSBSim
endif

#
# add the simulator directory to the make searchpath
#
VPATH = $(PAPARAZZI_SRC)/sw/simulator

NPSDIR = nps
nps.srcs += $(NPSDIR)/nps_main.c                 \
       $(NPSDIR)/nps_fdm_jsbsim.cpp              \
       $(NPSDIR)/nps_random.c                    \
       $(NPSDIR)/nps_sensors.c                   \
       $(NPSDIR)/nps_sensors_utils.c             \
       $(NPSDIR)/nps_sensor_gyro.c               \
       $(NPSDIR)/nps_sensor_accel.c              \
       $(NPSDIR)/nps_sensor_mag.c                \
       $(NPSDIR)/nps_sensor_baro.c               \
       $(NPSDIR)/nps_sensor_sonar.c              \
       $(NPSDIR)/nps_sensor_gps.c                \
       $(NPSDIR)/nps_electrical.c                \
       $(NPSDIR)/nps_atmosphere.c                \
       $(NPSDIR)/nps_radio_control.c             \
       $(NPSDIR)/nps_radio_control_joystick.c    \
       $(NPSDIR)/nps_radio_control_spektrum.c    \
       $(NPSDIR)/nps_autopilot_rotorcraft.c      \
       $(NPSDIR)/nps_ivy_common.c                \
       $(NPSDIR)/nps_ivy_rotorcraft.c            \
       $(NPSDIR)/nps_flightgear.c                \
       $(NPSDIR)/nps_ivy_mission_commands.c

# for geo mag calculation
nps.srcs += math/pprz_geodetic_wmm2015.c

include $(CFG_SHARED)/telemetry_ivy.makefile
nps.srcs += $(SRC_FIRMWARE)/rotorcraft_telemetry.c
nps.srcs += $(SRC_FIRMWARE)/datalink.c
