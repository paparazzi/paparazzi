

CC = gcc
CFLAGS = -std=c99 -I.. -I../.. -I../../../include -I../../booz -I../../../booz  -Wall
LDFLAGS = -lm

# imu wants airframe to fetch its neutrals
# ahrs wants airframe to fetch IMU_BODY_TO_IMU_ANGLES
CFLAGS += -I../../../../var/BOOZ2_A7


#CFLAGS += -DIMU_BODY_TO_IMU_PHI=0 -DIMU_BODY_TO_IMU_THETA=0 -DIMU_BODY_TO_IMU_PSI=0
# toulouse 0.51562740288882, -0.05707735220832, 0.85490967783446
CFLAGS += -DAHRS_H_X=0.51562740288882 -DAHRS_H_Y=-0.05707735220832 -DAHRS_H_Z=0.85490967783446

#CFLAGS += -DOUTPUT_IN_BODY_FRAME
CFLAGS += -DENABLE_MAG_UPDATE
CFLAGS += -DENABLE_ACCEL_UPDATE

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

clean:
	rm -f *~ run_ahrs_*_on_flight_log
