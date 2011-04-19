# Launch with "make Q=''" to get full command display
Q=@


CC = gcc
CFLAGS = -std=c99 -I.. -I../.. -I../../../include -I../../booz -I../../../booz  -Wall
LDFLAGS = -lm -lgsl -lgslcblas

CFLAGS +=

# imu wants airframe to fetch its neutrals
# ahrs wants airframe to fetch IMU_BODY_TO_IMU_ANGLES
#CFLAGS += -I../../../../var/BOOZ2_A7
CFLAGS += -I../../../../var/TA


#CFLAGS += -DIMU_BODY_TO_IMU_PHI=0 -DIMU_BODY_TO_IMU_THETA=0 -DIMU_BODY_TO_IMU_PSI=0
# toulouse 0.51562740288882, -0.05707735220832, 0.85490967783446
CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446

#CFLAGS += -DDISABLE_ALIGNEMENT
#CFLAGS += -DDISABLE_PROPAGATE
#CFLAGS += -DDISABLE_ACCEL_UPDATE
#CFLAGS += -DDISABLE_MAG_UPDATE
#CFLAGS += -DPERFECT_SENSORS
#CFLAGS += -DLOW_NOISE_THRESHOLD='ACCEL_BFP_OF_REAL(5.5)'
CFLAGS += -DLOW_NOISE_THRESHOLD=500000
CFLAGS += -DLOW_NOISE_TIME=2

SRCS= run_ahrs_on_flight_log.c                    \
      ../../math/pprz_trig_int.c                  \
      ../../subsystems/ahrs.c                     \
      ../../subsystems/ahrs/ahrs_aligner.c        \
      ../../subsystems/imu.c

all: run_ahrs_flq_on_flight_log run_ahrs_fcr_on_flight_log run_ahrs_ice_on_flight_log

run_ahrs_flq_on_flight_log: ../../subsystems/ahrs/ahrs_float_lkf_quat.c $(SRCS)
	$(CC) -DAHRS_TYPE=AHRS_TYPE_FLQ $(CFLAGS) -o $@ $^ $(LDFLAGS)

run_ahrs_fcr_on_flight_log: ../../subsystems/ahrs/ahrs_float_cmpl_rmat.c $(SRCS)
	$(CC) -DAHRS_TYPE=AHRS_TYPE_FCR $(CFLAGS) -o $@ $^ $(LDFLAGS)

run_ahrs_ice_on_flight_log: ../../subsystems/ahrs/ahrs_int_cmpl_euler.c $(SRCS)
	$(CC) -DAHRS_TYPE=AHRS_TYPE_ICE $(CFLAGS) -o $@ $^ $(LDFLAGS)

ifndef AHRS_TYPE
#AHRS_TYPE = AHRS_TYPE_ICE
AHRS_TYPE = AHRS_TYPE_ICQ
#AHRS_TYPE = AHRS_TYPE_FLQ
#AHRS_TYPE = AHRS_TYPE_FCR   # doesn't work - inverted accel
#AHRS_TYPE = AHRS_TYPE_FCR2
#AHRS_TYPE = AHRS_TYPE_FCQ
endif
ifndef PROPAGATE_LOW_PASS_RATES
PROPAGATE_LOW_PASS_RATES = 0
endif
ifndef GRAVITY_UPDATE_NORM_HEURISTIC
GRAVITY_UPDATE_NORM_HEURISTIC = 0
endif
ifndef GRAVITY_UPDATE_COORDINATED_TURN
GRAVITY_UPDATE_COORDINATED_TURN = 0
endif
ifndef MAG_UPDATE_YAW_ONLY
MAG_UPDATE_YAW_ONLY = 0
endif


AHRS_CFLAGS += -DAHRS_TYPE=$(AHRS_TYPE)
AHRS_CFLAGS += -DPERIODIC_FREQUENCY=512
AHRS_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512

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

AHRS_SRCS   += ../../subsystems/ahrs.c                      \
               ../../subsystems/ahrs/ahrs_aligner.c         \

ifeq ($(AHRS_TYPE), AHRS_TYPE_ICE)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl_euler.h\"
AHRS_CFLAGS += -DFACE_REINJ_1=1024
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_int_cmpl_euler.c 
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_ICQ)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_int_cmpl.h\"
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_int_cmpl.c 
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FLQ)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_lkf_quat.h\"
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_float_lkf_quat.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCR)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_dcm.h\"
AHRS_CFLAGS += -DINS_ROLL_NEUTRAL_DEFAULT=0
AHRS_CFLAGS += -DINS_PITCH_NEUTRAL_DEFAULT=0
AHRS_CFLAGS += -DAHRS_PROPAGATE_FREQUENCY=512
AHRS_CFLAGS += -DDCM_UPDATE_AFTER_PROPAGATE
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_float_dcm.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCR2)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_rmat.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_RMAT
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_float_cmpl_rmat.c
endif
ifeq ($(AHRS_TYPE), AHRS_TYPE_FCQ)
AHRS_CFLAGS += -DAHRS_TYPE_H=\"subsystems/ahrs/ahrs_float_cmpl_rmat.h\"
AHRS_CFLAGS += -DAHRS_PROPAGATE_QUAT
AHRS_SRCS   += ../../subsystems/ahrs/ahrs_float_cmpl_rmat.c
endif


RAOS_SRCS =      ./ahrs_on_synth.c		     \
	         ../../subsystems/imu.c              \
                 ../../subsystems/imu/imu_dummy.c    \
                 ../../math/pprz_trig_int.c          \
                 ../../../simulator/nps/nps_random.c


run_ahrs_on_synth: run_ahrs_on_synth.c $(RAOS_SRCS) $(AHRS_SRCS)
	@echo "Building run_ahrs_on_synth for $(AHRS_TYPE)"
	$(Q) $(CC) $(CFLAGS) $(AHRS_CFLAGS) -o $@ $^ $(LDFLAGS) 

IVY_CFLAGS=-g -O2 -Wall `pkg-config glib-2.0 --cflags`
IVY_LDFLAGS=`pkg-config glib-2.0 --libs` `pcre-config --libs` -lglibivy

run_ahrs_on_synth_ivy: run_ahrs_on_synth_ivy.c $(RAOS_SRCS) $(AHRS_SRCS)
	$(CC) $(CFLAGS) $(AHRS_CFLAGS) $(IVY_CFLAGS) -o $@ $^ $(LDFLAGS) $(IVY_LDFLAGS)





clean:
	@echo "cleaning ..."
	$(Q) rm -f *~ run_ahrs_*_on_flight_log run_ahrs_on_synth_ivy run_ahrs_on_synth
