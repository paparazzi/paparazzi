#ifndef BOOZ_SENSORS_MODEL_PARAMS_H
#define BOOZ_SENSORS_MODEL_PARAMS_H

/* 
 * Accelerometer 
 */
#define BSM_ACCEL_RESOLUTION      (1024 * 32)
/* ms-2 */
#define BSM_ACCEL_SENSITIVITY_XX -(1024. * 32.)/(2 * 6. * 9.81)
#define BSM_ACCEL_SENSITIVITY_YY  (1024. * 32.)/(2 * 6. * 9.81)
#define BSM_ACCEL_SENSITIVITY_ZZ  (1024. * 32.)/(2 * 6. * 9.81)
#define BSM_ACCEL_NEUTRAL_X       (538. * 32.)
#define BSM_ACCEL_NEUTRAL_Y       (506. * 32.)
#define BSM_ACCEL_NEUTRAL_Z       (506. * 32.)
/* m2s-4 */
#define BSM_ACCEL_NOISE_STD_DEV_X 2e-1
#define BSM_ACCEL_NOISE_STD_DEV_Y 2e-1
#define BSM_ACCEL_NOISE_STD_DEV_Z 2e-1
/* ms-2 */
#define BSM_ACCEL_BIAS_X          1e-3
#define BSM_ACCEL_BIAS_Y          1e-3
#define BSM_ACCEL_BIAS_Z          1e-3



/* 
 * Gyrometer 
 */
#define BSM_GYRO_RESOLUTION       65536
/* degres/s - nominal 300 */
#define BSM_GYRO_SENSITIVITY_PP   65536. / (2.*RadOfDeg(-413.41848));
#define BSM_GYRO_SENSITIVITY_QQ   65536. / (2.*RadOfDeg(-403.65564));
#define BSM_GYRO_SENSITIVITY_RR   65536. / (2.*RadOfDeg( 395.01929));

#define BSM_GYRO_NEUTRAL_P        65536. * 0.6238556;
#define BSM_GYRO_NEUTRAL_Q        65536. * 0.6242371;
#define BSM_GYRO_NEUTRAL_R        65536. * 0.6035156;

#define BSM_GYRO_NOISE_STD_DEV_P  RadOfDeg(.5)
#define BSM_GYRO_NOISE_STD_DEV_Q  RadOfDeg(.5)
#define BSM_GYRO_NOISE_STD_DEV_R  RadOfDeg(.5)

#define BSM_GYRO_BIAS_INITIAL_P  RadOfDeg(  .5)
#define BSM_GYRO_BIAS_INITIAL_Q  RadOfDeg(-0.5)
#define BSM_GYRO_BIAS_INITIAL_R  RadOfDeg(  .25)

#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_P RadOfDeg(5.e-1)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_Q RadOfDeg(5.e-1)
#define BSM_GYRO_BIAS_RANDOM_WALK_STD_DEV_R RadOfDeg(5.e-1)


/*
 *  Magnetometer
 */




/*
 *  Range meter
 */
#define BSM_RANGE_METER_RESOLUTION  (1024)
#define BSM_RANGE_METER_SENSITIVITY (1024. / 12.)
#define BSM_RANGE_METER_MAX_RANGE   (6. * BSM_RANGE_METER_SENSITIVITY)

/*
 *  Barometer
 */


/*
 *  GPS
 */



#endif /* BOOZ_SENSORS_MODEL_PARAMS_H */
