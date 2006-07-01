#ifndef AHRS_H
#define AHRS_H


#include <inttypes.h>

#define real_t  float
#define index_t uint8_t

extern real_t ahrs_pqr[3];
extern real_t q0, q1, q2, q3;
extern real_t ahrs_euler[3];
extern real_t bias_p, bias_q, bias_r;


extern void ahrs_init( const int16_t *mag );
extern void ahrs_state_update( void );
extern void ahrs_pitch_update( real_t pitch);
extern void ahrs_roll_update( real_t roll);
extern void ahrs_compass_update( real_t heading	);

extern real_t ahrs_roll_of_accel( real_t* accel_cal );
extern real_t ahrs_pitch_of_accel( real_t* accel_cal);
extern real_t ahrs_heading_of_mag( const int16_t *mag);

#endif /* AHRS_H */
