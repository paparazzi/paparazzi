#
# SITL Simulator
#

SIM_TYPE = JSBSIM

MY_JSBSIM_ROOT = /home/violato/enac/programs/JSBSim
MY_JSBSIM_LIB = /home/violato/enac/programs/install_jsbsim
SRC_BOOZ=booz
SRC_BOOZ_SIM = $(SRC_BOOZ)/sim


sim.ARCHDIR = $(ARCHI)
sim.ARCH = sitl
sim.TARGET = sim
sim.TARGETDIR = sim

sim.CFLAGS  += -DSITL
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lmeschach -lpcre -lglibivy
sim.CFLAGS  += -I$(SIMDIR) -I/usr/local/include -I$(MY_JSBSIM_LIB)/include/JSBSim
sim.LDFLAGS += -L$(MY_JSBSIM_LIB)/lib -lJSBSim

sim.CFLAGS += -I$(SRC_BOOZ) -I$(SRC_BOOZ_SIM) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator
sim.CFLAGS += -DJSBSIM_ROOT_DIR=\"/home/violato/enac/programs/JSBSim/\"

sim.srcs = $(SIMDIR)/nps_main.c                      \
	   $(SIMDIR)/nps_fdm_jsbsim.c                \
	   $(SIMDIR)/nps_sensors.c                   \
	   $(SIMDIR)/nps_sensor_gyro.c               \
	   $(SIMDIR)/nps_sensor_accel.c              \
	   $(SIMDIR)/nps_autopilot.c                 \

sim.srcs += $(SRC_BOOZ)/booz_trig_int.c
