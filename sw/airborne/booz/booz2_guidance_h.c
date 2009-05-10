#include "booz2_guidance_h.h"

#include "booz_ahrs.h"
#include "booz2_stabilization_rate.h"
#include "booz2_stabilization_attitude.h"
#include "booz2_stabilization_attitude_ref_traj_euler.h"
#include "booz2_fms.h"
#include "booz2_ins.h"
#include "booz2_navigation.h"

#include "airframe.h"
#include "radio_control.h"

uint8_t booz2_guidance_h_mode;

struct booz_ivect2 booz2_guidance_h_pos_sp;
int32_t            booz2_guidance_h_psi_sp;

struct booz_ivect2 booz2_guidance_h_pos_err;
struct booz_ivect2 booz2_guidance_h_speed_err;
struct booz_ivect2 booz2_guidance_h_pos_err_sum;

struct booz_ieuler booz2_guidance_h_rc_sp;
struct booz_ivect2 booz2_guidance_h_command_earth;
struct booz_ivect2 booz2_guidance_h_stick_earth_sp;
struct booz_ieuler booz2_guidance_h_command_body;

int32_t booz2_guidance_h_pgain;
int32_t booz2_guidance_h_dgain;
int32_t booz2_guidance_h_igain;


static inline void booz2_guidance_h_hover_run(void);
static inline void booz2_guidance_h_hover_enter(void);


void booz2_guidance_h_init(void) {

  booz2_guidance_h_mode = BOOZ2_GUIDANCE_H_MODE_KILL;
  booz2_guidance_h_psi_sp = 0;
  INT_VECT2_ZERO(booz2_guidance_h_pos_sp);
  BOOZ_IVECT2_ZERO(booz2_guidance_h_pos_err_sum);
  BOOZ_IEULER_ZERO(booz2_guidance_h_rc_sp);
  BOOZ_IEULER_ZERO(booz2_guidance_h_command_body);
  booz2_guidance_h_pgain = BOOZ2_GUIDANCE_H_PGAIN;
  booz2_guidance_h_igain = BOOZ2_GUIDANCE_H_IGAIN;
  booz2_guidance_h_dgain = BOOZ2_GUIDANCE_H_DGAIN;

}


void booz2_guidance_h_mode_changed(uint8_t new_mode) {
  if (new_mode == booz2_guidance_h_mode)
    return;

  switch ( booz2_guidance_h_mode ) {
	//      case BOOZ2_GUIDANCE_H_MODE_RATE:
	//	booz_stabilization_rate_exit();
	//	break;
  }
   
  switch (new_mode) {

  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz2_stabilization_attitude_enter();
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
  case BOOZ2_GUIDANCE_H_MODE_NAV:
    booz2_guidance_h_hover_enter();
    break;

  }

  booz2_guidance_h_mode = new_mode;
  
}

void booz2_guidance_h_read_rc(bool_t  in_flight) {
  
  switch ( booz2_guidance_h_mode ) {

  case BOOZ2_GUIDANCE_H_MODE_RATE:
    booz2_stabilization_rate_read_rc();
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz2_stabilization_attitude_read_rc(in_flight);
    if (booz_fms_on)
      BOOZ2_STABILIZATION_ATTITUDE_ADD_SP(booz_fms_input.h_sp.attitude);
    break;
  
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
    if (booz_fms_on && booz_fms_input.h_mode >= BOOZ2_GUIDANCE_H_MODE_HOVER)
      BOOZ2_FMS_SET_POS_SP(booz2_guidance_h_pos_sp,booz_stabilization_att_sp.psi);
    BOOZ2_STABILIZATION_ATTITUDE_READ_RC(booz2_guidance_h_rc_sp, in_flight);
    break;
  
  case BOOZ2_GUIDANCE_H_MODE_NAV:
    BOOZ2_STABILIZATION_ATTITUDE_READ_RC(booz2_guidance_h_rc_sp, in_flight);
    break;
  }

}


void booz2_guidance_h_run(bool_t  in_flight) {
  switch ( booz2_guidance_h_mode ) {

  case BOOZ2_GUIDANCE_H_MODE_RATE:
    booz2_stabilization_rate_run();
    break;

  case BOOZ2_GUIDANCE_H_MODE_ATTITUDE:
    booz2_stabilization_attitude_run(in_flight);
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_HOVER:
    booz2_guidance_h_hover_run();
    booz2_stabilization_attitude_run(in_flight);
    break;
    
  case BOOZ2_GUIDANCE_H_MODE_NAV:
    {
      if (horizontal_mode == HORIZONTAL_MODE_ATTITUDE) {
        booz_stabilization_att_sp.phi = 0;
        booz_stabilization_att_sp.theta = 0;
      }
      else {
        INT32_VECT2_NED_OF_ENU(booz2_guidance_h_pos_sp, booz2_navigation_carrot);
        booz2_guidance_h_psi_sp = (nav_heading << (ANGLE_REF_RES - INT32_ANGLE_FRAC));
        booz2_guidance_h_hover_run();
      }
      booz2_stabilization_attitude_run(in_flight);
      break;
    }
    
  }



}

#define MAX_POS_ERR   BOOZ_POS_I_OF_F(16.)
#define MAX_SPEED_ERR BOOZ_SPEED_I_OF_F(16.)
#define MAX_POS_ERR_SUM ((int32_t)(MAX_POS_ERR)<< 12)

// 15 degres
//#define MAX_BANK (65536)
#define MAX_BANK (98000)

static inline void  booz2_guidance_h_hover_run(void) {

  /* compute position error    */
  BOOZ_IVECT2_DIFF(booz2_guidance_h_pos_err, booz_ins_ltp_pos, booz2_guidance_h_pos_sp);
  /* saturate it               */
  BOOZ_IVECT2_STRIM(booz2_guidance_h_pos_err, -MAX_POS_ERR, MAX_POS_ERR);

  /* compute speed error    */
  BOOZ_IVECT2_COPY(booz2_guidance_h_speed_err, booz_ins_ltp_speed);
  /* saturate it               */
  BOOZ_IVECT2_STRIM(booz2_guidance_h_speed_err, -MAX_SPEED_ERR, MAX_SPEED_ERR);
  
  /* update pos error integral */
  BOOZ_IVECT2_ADD(booz2_guidance_h_pos_err_sum, booz2_guidance_h_pos_err);
  /* saturate it               */
  BOOZ_IVECT2_STRIM(booz2_guidance_h_pos_err_sum, -MAX_POS_ERR_SUM, MAX_POS_ERR_SUM);
		    
  /* run PID */
  // cmd_earth < 15.17
  booz2_guidance_h_command_earth.x = (booz2_guidance_h_pgain<<1)  * booz2_guidance_h_pos_err.x +
                                     booz2_guidance_h_dgain * (booz2_guidance_h_speed_err.x>>9) +
                                      booz2_guidance_h_igain * (booz2_guidance_h_pos_err_sum.x >> 12); 
  booz2_guidance_h_command_earth.y = (booz2_guidance_h_pgain<<1)  * booz2_guidance_h_pos_err.y +
                                     booz2_guidance_h_dgain *( booz2_guidance_h_speed_err.y>>9) +
		                      booz2_guidance_h_igain * (booz2_guidance_h_pos_err_sum.y >> 12); 

  BOOZ_IVECT2_STRIM(booz2_guidance_h_command_earth, -MAX_BANK, MAX_BANK);

  /* Rotate to body frame */
  int32_t s_psi, c_psi;
  BOOZ_ISIN(s_psi, booz_ahrs.ltp_to_body_euler.psi);	
  BOOZ_ICOS(c_psi, booz_ahrs.ltp_to_body_euler.psi);	


  // ITRIG_RES - 2: 100mm erreur, gain 100 -> 10000 command | 2 degres = 36000, so multiply by 4
  booz2_guidance_h_command_body.phi = 
      ( - s_psi * booz2_guidance_h_command_earth.x + c_psi * booz2_guidance_h_command_earth.y)
    >> (ITRIG_RES - 2);
  booz2_guidance_h_command_body.theta = 
    - ( c_psi * booz2_guidance_h_command_earth.x + s_psi * booz2_guidance_h_command_earth.y)
    >> (ITRIG_RES - 2);


  booz2_guidance_h_command_body.phi   += booz2_guidance_h_rc_sp.phi;
  booz2_guidance_h_command_body.theta += booz2_guidance_h_rc_sp.theta;
  booz2_guidance_h_command_body.psi    = booz2_guidance_h_psi_sp + booz2_guidance_h_rc_sp.psi;
  ANGLE_REF_NORMALIZE(booz2_guidance_h_command_body.psi);

  BOOZ_IEULER_COPY(booz_stabilization_att_sp, booz2_guidance_h_command_body);

}

static inline void booz2_guidance_h_hover_enter(void) {

  BOOZ_IVECT2_COPY(booz2_guidance_h_pos_sp, booz_ins_ltp_pos);

  BOOZ2_STABILIZATION_ATTITUDE_RESET_PSI_REF( booz2_guidance_h_rc_sp );

  BOOZ_IVECT2_ZERO(booz2_guidance_h_pos_err_sum);

  BOOZ2_FMS_POS_INIT(booz2_guidance_h_pos_sp,booz2_guidance_h_rc_sp.psi);
}
