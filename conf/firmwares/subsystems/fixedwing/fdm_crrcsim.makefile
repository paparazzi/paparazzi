# Hey Emacs, this is a -*- makefile -*-

#
# SITL Simulator based on CRRCSIM (MNAV inputdev)
#

SRC_FIRMWARE=firmwares/fixedwing

SRC_BOARD=boards/$(BOARD)

nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

# add normal ap and fbw sources define in autopilot.makefile
nps.CFLAGS  += $(fbw_CFLAGS) $(ap_CFLAGS)
nps.srcs    += $(fbw_srcs) $(ap_srcs)

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.CFLAGS  += $(shell pkg-config glib-2.0 --cflags)
nps.LDFLAGS += $(shell pkg-config glib-2.0 --libs) -lm -lglibivy $(shell pcre-config --libs) -lgsl -lgslcblas
nps.CFLAGS  += -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I$(PAPARAZZI_SRC)/sw/simulator -I$(PAPARAZZI_SRC)/sw/simulator/nps -I$(PAPARAZZI_HOME)/conf/simulator/nps
nps.LDFLAGS += $(shell sdl-config --libs)

#
# add the simulator directory to the make searchpath
#
VPATH = $(PAPARAZZI_SRC)/sw/simulator

NPSDIR = nps
nps.srcs += $(NPSDIR)/nps_main.c                 \
       $(NPSDIR)/nps_fdm_crrcsim.c               \
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
       $(NPSDIR)/nps_autopilot_fixedwing.c       \
       $(NPSDIR)/nps_ivy.c                       \
       $(NPSDIR)/nps_flightgear.c                \


include $(CFG_SHARED)/telemetry_transparent_udp.makefile
nps.srcs += $(SRC_FIRMWARE)/datalink.c
nps.srcs += $(SRC_FIRMWARE)/ap_downlink.c $(SRC_FIRMWARE)/fbw_downlink.c
