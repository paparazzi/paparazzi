# Hey Emacs, this is a -*- makefile -*-

#
# NPS SITL Simulator
#
# still needs a FDM backend to be specified, e.g.
# <subsystem name="fdm" type="jsbsim"/>
#

nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.CFLAGS  += $(shell pkg-config glib-2.0 --cflags)
nps.LDFLAGS += $(shell pkg-config glib-2.0 --libs) -lm -lglibivy $(shell pcre-config --libs) -lgsl -lgslcblas
nps.CFLAGS  += -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I$(PAPARAZZI_SRC)/sw/simulator -I$(PAPARAZZI_SRC)/sw/simulator/nps -I$(PAPARAZZI_HOME)/conf/simulator/nps
nps.LDFLAGS += $(shell sdl-config --libs)


#
# add the simulator directory to the make searchpath
#
VPATH += $(PAPARAZZI_SRC)/sw/simulator

NPSDIR = nps
nps.srcs += $(NPSDIR)/nps_main.c                 \
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
       $(NPSDIR)/nps_radio_control.c             \
       $(NPSDIR)/nps_radio_control_joystick.c    \
       $(NPSDIR)/nps_radio_control_spektrum.c    \
       $(NPSDIR)/nps_ivy.c                       \
       $(NPSDIR)/nps_flightgear.c

# for geo mag calculation
nps.srcs += math/pprz_geodetic_wmm2015.c

ifeq ($(TARGET), nps)
include $(CFG_SHARED)/telemetry_transparent_udp.makefile
endif
