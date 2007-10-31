#ifndef MULTITILT_H
#define MULTITILT_H

#include "std.h"

#define MT_STATUS_UNINIT       0
#define MT_STATUS_RUNNING      1
#define MT_STATUS_CRASHED      2

extern uint8_t mtt_status;

extern float mtt_phi; 
extern float mtt_p;
extern float mtt_bp;
extern float mtt_P_phi[2][2];

extern float mtt_theta; 
extern float mtt_q;
extern float mtt_bq;
extern float mtt_P_theta[2][2];

extern float mtt_psi;
extern float mtt_r;
extern float mtt_br;


extern void multitilt_init(void);
extern void multitilt_start( const float* accel, const float* gyro, const int16_t* mag);
extern void multitilt_predict( const float* gyro );
extern void multitilt_update( const float* accel, const int16_t* mag );


#endif /* MULTITILT_H */
