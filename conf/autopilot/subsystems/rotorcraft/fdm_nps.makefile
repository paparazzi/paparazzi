#
# SITL Simulator
#

SIM_TYPE = JSBSIM

JSBSIM_ROOT = /opt/jsbsim
JSBSIM_INC = $(JSBSIM_ROOT)/include/JSBSim
JSBSIM_LIB = $(JSBSIM_ROOT)/lib

SRC_BOOZ=booz
SRC_BOOZ_SIM = $(SRC_BOOZ)/arch/sim

SRC_FIRMWARE=firmwares/rotorcraft

SRC_BOARD=boards/$(BOARD)

NPSDIR = $(SIMDIR)/nps


sim.ARCHDIR = $(ARCH)

sim.CFLAGS  += -DSITL -DNPS
sim.CFLAGS  += `pkg-config glib-2.0 --cflags` -I /usr/include/meschach
sim.LDFLAGS += `pkg-config glib-2.0 --libs` -lm -lpcre -lglibivy -lgsl -lgslcblas
sim.CFLAGS  += -I$(NPSDIR) -I$(SRC_FIRMWARE) -I$(SRC_BOOZ) -I$(SRC_BOOZ_SIM) -I$(SRC_BOARD) -I../simulator -I$(PAPARAZZI_HOME)/conf/simulator/nps

# use the paparazzi-jsbsim package if it is installed, otherwise look for JSBsim under /opt/jsbsim
ifndef JSBSIM_PKG
JSBSIM_PKG = $(shell pkg-config JSBSim --exists && echo 'yes')
endif
ifeq ($(JSBSIM_PKG), yes)
	sim.CFLAGS  += `pkg-config JSBSim --cflags`
	sim.LDFLAGS += `pkg-config JSBSim --libs`
else
	JSBSIM_PKG = no
	sim.CFLAGS  += -I$(JSBSIM_INC)
	sim.LDFLAGS += -L$(JSBSIM_LIB) -lJSBSim
endif


sim.srcs = $(NPSDIR)/nps_main.c                      \
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
       $(NPSDIR)/nps_autopilot_booz.c            \
       $(NPSDIR)/nps_ivy.c                       \
       $(NPSDIR)/nps_flightgear.c                \


sim.srcs += math/pprz_trig_int.c             \
            math/pprz_geodetic_float.c       \
            math/pprz_geodetic_double.c      \



sim.CFLAGS += -DBOARD_CONFIG=$(BOARD_CFG)

sim.srcs   += firmwares/rotorcraft/main.c
sim.srcs   += mcu.c
sim.srcs   += $(SRC_ARCH)/mcu_arch.c

sim.CFLAGS += -DPERIODIC_TASK_PERIOD='SYS_TICS_OF_SEC((1./512.))'
# -DTIME_LED=1
#sim.CFLAGS += -DUSE_LED
sim.srcs += sys_time.c

sim.srcs += subsystems/settings.c
sim.srcs += $(SRC_ARCH)/subsystems/settings_arch.c

sim.CFLAGS += -DDOWNLINK -DDOWNLINK_TRANSPORT=IvyTransport
sim.srcs += $(SRC_FIRMWARE)/telemetry.c \
            downlink.c \
            $(SRC_ARCH)/ivy_transport.c

sim.srcs   += $(SRC_BOOZ)/booz2_commands.c

sim.srcs += $(SRC_FIRMWARE)/datalink.c

#
#
#


sim.CFLAGS += -DROTORCRAFT_BARO_LED=2
sim.srcs += $(SRC_BOARD)/baro_board.c

sim.CFLAGS += -DUSE_ADC
sim.srcs   += $(SRC_ARCH)/mcu_periph/adc_arch.c
sim.srcs   += subsystems/electrical.c
# baro has variable offset amplifier on booz board
#sim.CFLAGS += -DUSE_DAC
#sim.srcs   += $(SRC_ARCH)/mcu_periph/dac_arch.c


#sim.CFLAGS += -DIMU_TYPE_H=\"imu/imu_b2.h\"
#sim.CFLAGS += -DIMU_B2_VERSION_1_1

sim.srcs += $(SRC_FIRMWARE)/autopilot.c

#
# in makefile section of airframe xml
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_lkf.makefile
# or
# include $(CFG_BOOZ)/subsystems/booz2_ahrs_cmpl.makefile
#

sim.srcs += $(SRC_FIRMWARE)/stabilization.c
sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_rate.c

NUM_TYPE=integer
#NUM_TYPE=float

STAB_TYPE=euler
#STAB_TYPE=quaternion

ifeq ($(NUM_TYPE), integer)
  sim.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_INT
  sim.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_int.h\"
  ifeq ($(STAB_TYPE), euler)
    sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_int.h\"
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_int.c
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_int.c
  else ifeq ($(STAB_TYPE), quaternion)
    sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_int.h\"
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_int.c
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_int.c
  endif
else ifeq ($(NUM_TYPE), float)
  sim.CFLAGS += -DSTABILISATION_ATTITUDE_TYPE_FLOAT
  sim.CFLAGS += -DSTABILISATION_ATTITUDE_H=\"stabilization/stabilization_attitude_float.h\"
  ifeq ($(STAB_TYPE), euler)
    sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_euler_float.h\"
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_euler_float.c
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_euler_float.c
  else ifeq ($(STAB_TYPE), quaternion)
    sim.CFLAGS += -DSTABILISATION_ATTITUDE_REF_H=\"stabilization/stabilization_attitude_ref_quat_float.h\"
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_ref_quat_float.c
    sim.srcs += $(SRC_FIRMWARE)/stabilization/stabilization_attitude_quat_float.c
  endif
endif

sim.CFLAGS += -DUSE_NAVIGATION
sim.srcs += $(SRC_FIRMWARE)/guidance/guidance_h.c
sim.srcs += $(SRC_FIRMWARE)/guidance/guidance_v.c
sim.srcs += math/pprz_geodetic_int.c
sim.srcs += $(SRC_SUBSYSTEMS)/ins.c

#  vertical filter float version
sim.srcs += $(SRC_SUBSYSTEMS)/ins/vf_float.c
sim.CFLAGS += -DUSE_VFF -DDT_VFILTER='(1./512.)'

#
# INS choice
#
# include ins_hff.makefile
# or
# nothing
#


sim.srcs += $(SRC_FIRMWARE)/navigation.c
