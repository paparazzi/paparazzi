# Hey Emacs, this is a -*- makefile -*-

#
# SITL Simulator
#

JSBSIM_ROOT ?= /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_FIRMWARE=firmwares/rotorcraft

SRC_BOARD=boards/$(BOARD)

NPSDIR = $(SIMDIR)/nps


nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.CFLAGS  += $(shell pkg-config glib-2.0 --cflags)
nps.LDFLAGS += $(shell pkg-config glib-2.0 --libs) -lm -lglibivy -lpcre -lgsl -lgslcblas
nps.CFLAGS  += -I$(NPSDIR) -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator/nps
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


nps.srcs += $(NPSDIR)/nps_main.c                      \
       $(NPSDIR)/nps_fdm_jsbsim.c                \
       $(NPSDIR)/nps_random.c                    \
       $(NPSDIR)/nps_sensors.c                   \
       $(NPSDIR)/nps_sensors_utils.c             \
       $(NPSDIR)/nps_sensor_gyro.c               \
       $(NPSDIR)/nps_sensor_accel.c              \
       $(NPSDIR)/nps_sensor_mag.c                \
       $(NPSDIR)/nps_sensor_baro.c               \
       $(NPSDIR)/nps_sensor_gps.c                \
       $(NPSDIR)/nps_radio_control.c             \
       $(NPSDIR)/nps_radio_control_joystick.c    \
       $(NPSDIR)/nps_radio_control_spektrum.c    \
       $(NPSDIR)/nps_autopilot_rotorcraft.c            \
       $(NPSDIR)/nps_ivy.c                       \
       $(NPSDIR)/nps_flightgear.c                \



nps.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG) -DPERIPHERALS_AUTO_INIT

nps.srcs   += firmwares/rotorcraft/main.c
nps.srcs   += mcu.c
nps.srcs   += $(SRC_ARCH)/mcu_arch.c

nps.srcs += mcu_periph/i2c.c
nps.srcs += $(SRC_ARCH)/mcu_periph/i2c_arch.c


PERIODIC_FREQUENCY ?= 512
TELEMETRY_FREQUENCY ?= 60
nps.CFLAGS += -DPERIODIC_FREQUENCY=$(PERIODIC_FREQUENCY)
nps.CFLAGS += -DTELEMETRY_FREQUENCY=$(TELEMETRY_FREQUENCY)
#nps.CFLAGS += -DUSE_LED
nps.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

nps.srcs += subsystems/settings.c
nps.srcs += $(SRC_ARCH)/subsystems/settings_arch.c

nps.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
nps.srcs += $(SRC_FIRMWARE)/telemetry.c \
            subsystems/datalink/downlink.c \
            $(SRC_ARCH)/ivy_transport.c

nps.srcs   += subsystems/actuators.c
nps.srcs   += subsystems/commands.c

nps.srcs += $(SRC_FIRMWARE)/datalink.c

#
# Math functions
#
nps.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c math/pprz_orientation_conversion.c

nps.CFLAGS += -DROTORCRAFT_BARO_LED=2
nps.srcs += $(SRC_BOARD)/baro_board.c

nps.CFLAGS += -DUSE_ADC
nps.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
nps.srcs   += subsystems/electrical.c

nps.srcs += $(SRC_FIRMWARE)/autopilot.c

nps.srcs += state.c

#
# in makefile section of airframe xml
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_lkf.makefile
# or
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_cmpl.makefile
#

nps.srcs += $(SRC_FIRMWARE)/stabilization.c
nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c
nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_none.c

nps.CFLAGS += -DUSE_NAVIGATION
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_h_ref.c
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_v_ref.c
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_v_adapt.c

#
# INS choice
#
# include subsystems/rotorcraft/ins.makefile
# or
# include subsystems/rotorcraft/ins_extended.makefile
#
# extra:
# include subsystems/rotorcraft/ins_hff.makefile
#

nps.srcs += $(SRC_FIRMWARE)/navigation.c
nps.srcs += $(SRC_SUBSYSTEMS)/navigation/common_flight_plan.c
