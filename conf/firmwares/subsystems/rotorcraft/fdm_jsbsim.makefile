# Hey Emacs, this is a -*- makefile -*-

#
# SITL Simulator
#

JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_FIRMWARE=firmwares/rotorcraft

SRC_BOARD=boards/$(BOARD)

NPSDIR = $(SIMDIR)/nps


nps.ARCHDIR = sim

# include Makefile.nps instead of Makefile.sim
nps.MAKEFILE = nps

nps.CFLAGS  += -DSITL -DUSE_NPS
nps.CFLAGS  += `pkg-config glib-2.0 --cflags`
nps.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lglibivy -lgsl -lgslcblas
nps.CFLAGS  += -I$(NPSDIR) -I$(SRC_FIRMWARE) -I$(SRC_BOARD) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator/nps
nps.LDFLAGS += `sdl-config --libs`

# use the paparazzi-jsbsim package if it is installed, otherwise look for JSBsim under /opt/jsbsim
ifndef JSBSIM_PKG
JSBSIM_PKG = $(shell pkg-config JSBSim --exists && echo 'yes')
endif
ifeq ($(JSBSIM_PKG), yes)
	nps.CFLAGS  += `pkg-config JSBSim --cflags`
	nps.LDFLAGS += `pkg-config JSBSim --libs`
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



nps.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

nps.srcs   += firmwares/rotorcraft/main.c
nps.srcs   += mcu.c
nps.srcs   += $(SRC_ARCH)/mcu_arch.c

ifeq ($(TARGET), nps)
  include $(CFG_SHARED)/i2c_select.makefile
endif


nps.CFLAGS += -DPERIODIC_FREQUENCY=512
#nps.CFLAGS += -DUSE_LED
nps.srcs += mcu_periph/sys_time.c $(SRC_ARCH)/mcu_periph/sys_time_arch.c

nps.srcs += subsystems/settings.c
nps.srcs += $(SRC_ARCH)/subsystems/settings_arch.c

nps.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
nps.srcs += $(SRC_FIRMWARE)/telemetry.c \
            subsystems/datalink/downlink.c \
            $(SRC_ARCH)/ivy_transport.c

nps.srcs   += $(SRC_FIRMWARE)/commands.c

nps.srcs += $(SRC_FIRMWARE)/datalink.c

#
# Math functions
#
nps.srcs += math/pprz_geodetic_int.c math/pprz_geodetic_float.c math/pprz_geodetic_double.c math/pprz_trig_int.c

nps.CFLAGS += -DROTORCRAFT_BARO_LED=2
nps.srcs += $(SRC_BOARD)/baro_board.c

nps.CFLAGS += -DUSE_ADC
nps.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
nps.srcs   += subsystems/electrical.c
# baro has variable offset amplifier on booz board
#nps.CFLAGS += -DUSE_DAC
#nps.srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c


#nps.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
#nps.CFLAGS += -DIMU_B2_VERSION_1_1

nps.srcs += $(SRC_FIRMWARE)/autopilot.c

#
# in makefile section of airframe xml
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_lkf.makefile
# or
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_cmpl.makefile
#

nps.srcs += $(SRC_FIRMWARE)/stabilization.c
nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c
nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_none.c


NUM_TYPE=integer
#NUM_TYPE=float

STAB_TYPE=euler
#STAB_TYPE=quaternion

ifeq ($(NUM_TYPE), integer)
  nps.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
  nps.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
  ifeq ($(STAB_TYPE), euler)
    nps.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c
  else ifeq ($(STAB_TYPE), quaternion)
    nps.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_int.h\"
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_int.c
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_int.c
  endif
else ifeq ($(NUM_TYPE), float)
  nps.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_FLOAT
  nps.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_float.h\"
  ifeq ($(STAB_TYPE), euler)
    nps.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_float.h\"
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_float.c
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_float.c
  else ifeq ($(STAB_TYPE), quaternion)
    nps.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_float.h\"
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_float.c
    nps.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_float.c
  endif
endif

nps.CFLAGS += -DUSE_NAVIGATION
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
nps.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
nps.srcs += $(SRC_SUBSYSTEMS)/ins.c

#  vertical filter float version
nps.srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
nps.CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./512.)'

#
# INS choice
#
# include ins_hff.makefile
# or
# nothing
#


nps.srcs += $(SRC_FIRMWARE)/navigation.c
nps.srcs += $(SRC_SUBSYSTEMS)/navigation/common_flight_plan.c
