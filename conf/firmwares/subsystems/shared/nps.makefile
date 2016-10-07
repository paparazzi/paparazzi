# Hey Emacs, this is a -*- makefile -*-

#
# NPS Simulator
#
# Common makefile for both SITL/HITL simulation
#
# still needs a FDM backend to be specified, e.g.
# <subsystem name="fdm" type="jsbsim"/>
#

USE_HITL ?= 0

nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.LDFLAGS += -lm -livy $(shell pcre-config --libs) -lgsl -lgslcblas

# detect system arch and include rt and pthread library only on linux
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Linux)
  nps.LDFLAGS += -lrt -pthread
endif

nps.CFLAGS  += -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I$(PAPARAZZI_SRC)/sw/simulator -I$(PAPARAZZI_SRC)/sw/simulator/nps -I$(PAPARAZZI_HOME)/conf/simulator/nps

# sdl needed for joystick input
nps.LDFLAGS += $(shell sdl-config --libs)


#
# add the simulator directory to the make searchpath
#
VPATH += $(PAPARAZZI_SRC)/sw/simulator

NPSDIR = nps
nps.srcs +=                                      \
       $(NPSDIR)/nps_random.c                    \
       $(NPSDIR)/nps_sensors.c                   \
       $(NPSDIR)/nps_sensors_utils.c             \
       $(NPSDIR)/nps_sensor_gyro.c               \
       $(NPSDIR)/nps_sensor_accel.c              \
       $(NPSDIR)/nps_sensor_mag.c                \
       $(NPSDIR)/nps_sensor_baro.c               \
       $(NPSDIR)/nps_sensor_sonar.c              \
       $(NPSDIR)/nps_sensor_gps.c                \
       $(NPSDIR)/nps_sensor_airspeed.c           \
       $(NPSDIR)/nps_sensor_temperature.c        \
       $(NPSDIR)/nps_electrical.c                \
       $(NPSDIR)/nps_atmosphere.c                \
       $(NPSDIR)/nps_ivy.c                       \
       $(NPSDIR)/nps_flightgear.c                \
       $(NPSDIR)/nps_radio_control.c             \
       $(NPSDIR)/nps_radio_control_joystick.c    \
       $(NPSDIR)/nps_radio_control_spektrum.c    \
       $(NPSDIR)/nps_main_common.c

ifeq ($(USE_HITL),1)
include $(CFG_SHARED)/nps_hitl.makefile
else
include $(CFG_SHARED)/nps_sitl.makefile
endif

# for geo mag calculation
nps.srcs += math/pprz_geodetic_wmm2015.c

