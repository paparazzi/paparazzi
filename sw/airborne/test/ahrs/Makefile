# Launch with "make Q=''" to get full command display
Q=@


CC = gcc
CFLAGS = -std=c99 -I.. -I../.. -I../../../include  -Wall
LDFLAGS = -lm -lgsl -lgslcblas

CFLAGS +=

# imu wants airframe to fetch its neutrals
# ahrs wants airframe to fetch IMU_BODY_TO_IMU_ANGLES
# have a fake one in ./generated/airframe.h
CFLAGS += -I.

#CFLAGS += -DIMU_BODY_TO_IMU_PHI=0 -DIMU_BODY_TO_IMU_THETA=0 -DIMU_BODY_TO_IMU_PSI=0
# toulouse 0.51562740288882, -0.05707735220832, 0.85490967783446
CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446

#CFLAGS += -DDISABLE_ALIGNEMENT
#CFLAGS += -DDISABLE_PROPAGATE
#CFLAGS += -DDISABLE_ACCEL_UPDATE
#CFLAGS += -DPERFECT_SENSORS
#CFLAGS += -DLOW_NOISE_THRESHOLD='ACCEL_BFP_OF_REAL(5.5)'
CFLAGS += -DLOW_NOISE_THRESHOLD=500000
CFLAGS += -DLOW_NOISE_TIME=2

FREQUENCY ?= 512

CFLAGS += -DPERIODIC_FREQUENCY=$(FREQUENCY)
CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=$(FREQUENCY)
CFLAGS += -DAHRS_CORRECT_FREQUENCY=$(FREQUENCY)
CFLAGS += -DAHRS_MAG_CORRECT_FREQUENCY=50

SRCS= run_ahrs_on_flight_log.c                    \
	  ../../math/pprz_trig_int.c                  \
	  ../../modules/ahrs/ahrs.c                     \
	  ../../modules/ahrs/ahrs_aligner.c        \
	  ../../modules/imu/imu.c                      \
      ../../state.c                               \
      ../../math/pprz_orientation_conversion.c    \
      ../../math/pprz_geodetic_double.c           \
      ../../math/pprz_geodetic_float.c            \
      ../../math/pprz_geodetic_int.c              \

all: run_ahrs_fcr_on_flight_log run_ahrs_ice_on_flight_log

#run_ahrs_flq_on_flight_log: ../../modules/ahrs/ahrs_float_lkf_quat.c $(SRCS)
#	$(CC) -DAHRS_TYPE=AHRS_TYPE_FLQ $(CFLAGS) -o $@ $^ $(LDFLAGS)

run_ahrs_fcr_on_flight_log: ../../modules/ahrs/ahrs_float_cmpl.c $(SRCS)
	$(CC) -DAHRS_TYPE=AHRS_TYPE_FCR -DAHRS_PROPAGATE_QUAT $(CFLAGS) -o $@ $^ $(LDFLAGS)

run_ahrs_ice_on_flight_log: ../../modules/ahrs/ahrs_int_cmpl_euler.c $(SRCS)
	$(CC) -DAHRS_TYPE=AHRS_TYPE_ICE $(CFLAGS) -o $@ $^ $(LDFLAGS)

ifndef AHRS_TYPE
#AHRS_TYPE = AHRS_TYPE_ICE
AHRS_TYPE = AHRS_TYPE_ICQ
#AHRS_TYPE = AHRS_TYPE_FLQ
#AHRS_TYPE = AHRS_TYPE_FCR   # doesn't work - inverted accel
#AHRS_TYPE = AHRS_TYPE_FCR2
#AHRS_TYPE = AHRS_TYPE_FCQ
endif

PROPAGATE_LOW_PASS_RATES ?= 0
GRAVITY_UPDATE_NORM_HEURISTIC ?= 0
GRAVITY_UPDATE_COORDINATED_TURN ?= 0
MAG_UPDATE_YAW_ONLY ?= 0
DISABLE_MAG_UPDATE ?= 0
USE_GPS_HEADING ?= 0

AHRS_CFLAGS += -DAHRS_TYPE=$(AHRS_TYPE)

ifeq ($(PROPAGATE_LOW_PASS_RATES), 1)
AHRS_CFLAGS += -DAHRS_PROPAGATE_LOW_PASS_RATES
endif
ifeq ($(GRAVITY_UPDATE_NORM_HEURISTIC), 1)
AHRS_CFLAGS += -DAHRS_GRAVITY_UPDATE_NORM_HEURISTIC
endif
ifeq ($(GRAVITY_UPDATE_COORDINATED_TURN), 1)
AHRS_CFLAGS += -DAHRS_GRAVITY_UPDATE_COORDINATED_TURN
endif
ifeq ($(MAG_UPDATE_YAW_ONLY), 1)
AHRS_CFLAGS += -DAHRS_MAG_UPDATE_YAW_ONLY
endif
ifeq ($(USE_GPS_HEADING), 1)
AHRS_CFLAGS += -DAHRS_USE_GPS_HEADING
endif
ifeq ($(DISABLE_MAG_UPDATE), 1)
AHRS_CFLAGS += -DDISABLE_MAG_UPDATE
endif
ifeq ($(USE_AHRS_GPS_ACCELERATIONS), 1)
AHRS_CFLAGS += -DUSE_AHRS_GPS_ACCELERATIONS
endif


AHRS_SRCS   += ../../modules/ahrs/ahrs.c                      \
			   ../../modules/ahrs/ahrs_aligner.c         \

ifeq ($(AHRS_TYPE), AHRS_TYPE_ICE)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"modules/ahrs/ahrs_int_cmpl_euler.h\"
AHRS_CFLAGS += -DFACE_REINJ_1=1024
AHRS_SRCS   += ../../modules/ahrs/ahrs_int_cmpl_euler.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_ICQ)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"modules/ahrs/ahrs_int_cmpl_quat.h\"
AHRS_SRCS   += ../../modules/ahrs/ahrs_int_cmpl_quat.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCR)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"modules/ahrs/ahrs_float_dcm.h\"
AHRS_SRCS   += ../../modules/ahrs/ahrs_float_dcm.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCR2)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"modules/ahrs/ahrs_float_cmpl.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_RMAT
AHRS_SRCS   += ../../modules/ahrs/ahrs_float_cmpl.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCQ)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"modules/ahrs/ahrs_float_cmpl.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_QUAT
AHRS_SRCS   += ../../modules/ahrs/ahrs_float_cmpl.c
endif


RAOS_SRCS =  ./ahrs_on_synth.c			 \
			 ../../state.c \
			 ../../modules/imu/imu.c              \
			 ../../modules/imu/imu_dummy.c    \
			 ../../math/pprz_trig_int.c          \
			 ../../math/pprz_orientation_conversion.c \
			 ../../math/pprz_geodetic_int.c \
			 ../../math/pprz_geodetic_float.c \
			 ../../math/pprz_geodetic_double.c \
			 ../../../simulator/nps/nps_random.c


run_ahrs_on_synth: run_ahrs_on_synth.c $(RAOS_SRCS) $(AHRS_SRCS)
	@echo "Building run_ahrs_on_synth for $(AHRS_TYPE)"
	$(Q) $(CC) $(CFLAGS) $(AHRS_CFLAGS) -o $@ $^ $(LDFLAGS)

IVY_CFLAGS=-g -O2 -Wall $(shell pkg-config glib-2.0 --cflags)
IVY_LDFLAGS=$(shell pkg-config glib-2.0 --libs) -lglibivy

run_ahrs_on_synth_ivy: run_ahrs_on_synth_ivy.c $(RAOS_SRCS) $(AHRS_SRCS)
	$(CC) $(CFLAGS) $(AHRS_CFLAGS) $(IVY_CFLAGS) -o $@ $^ $(LDFLAGS) $(IVY_LDFLAGS)





clean:
	@echo "cleaning ..."
	$(Q) rm -f *~ run_ahrs_*_on_flight_log run_ahrs_on_synth_ivy run_ahrs_on_synth
