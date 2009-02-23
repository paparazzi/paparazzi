#ifndef BOOZ2_GUIDANCE_H_H
#define BOOZ2_GUIDANCE_H_H


#include "booz_geometry_int.h"

#define BOOZ2_GUIDANCE_H_MODE_KILL      0
#define BOOZ2_GUIDANCE_H_MODE_RATE      1
#define BOOZ2_GUIDANCE_H_MODE_ATTITUDE  2
#define BOOZ2_GUIDANCE_H_MODE_HOVER     3
#define BOOZ2_GUIDANCE_H_MODE_NAV       4


extern uint8_t booz2_guidance_h_mode;

/* horizontal setpoint in NED */
/* Q_int32_xx_8        */
extern struct booz_ivect2 booz2_guidance_h_pos_sp;
extern int32_t            booz2_guidance_h_psi_sp;

extern struct booz_ivect2 booz2_guidance_h_pos_err;
extern struct booz_ivect2 booz2_guidance_h_speed_err;
extern struct booz_ivect2 booz2_guidance_h_pos_err_sum;

extern struct booz_ieuler booz2_guidance_h_rc_sp;
extern struct booz_ivect2 booz2_guidance_h_command_earth;
extern struct booz_ieuler booz2_guidance_h_command_body;

extern int32_t booz2_guidance_h_pgain;
extern int32_t booz2_guidance_h_dgain;
extern int32_t booz2_guidance_h_igain;


extern void booz2_guidance_h_init(void);
extern void booz2_guidance_h_mode_changed(uint8_t new_mode);
extern void booz2_guidance_h_read_rc(bool_t  in_flight);
extern void booz2_guidance_h_run(bool_t  in_flight);


#define booz2_guidance_h_SetKi(_val) {			\
    booz2_guidance_h_igain = _val;			\
    BOOZ_IVECT2_ZERO(booz2_guidance_h_pos_err_sum);	\
  }

#endif /* BOOZ2_GUIDANCE_H_H */
