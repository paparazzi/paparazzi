#ifndef GPS_SIM_HITL_H
#define GPS_SIM_HITL_H

#include "std.h"
#include "subsystems/ins.h"
#include "guidance/guidance_h.h"
#include "guidance/guidance_v.h"

#define GPS_NB_CHANNELS 16

extern bool_t gps_available;
extern uint32_t gps_sim_hitl_timer;

#define GpsEvent(_sol_available_callback) {                                                         \
    if(SysTimeTimer(gps_sim_hitl_timer) > 100000) {                                                 \
      SysTimeTimerStart(gps_sim_hitl_timer);                                                        \
      if(ins_impl.ltp_initialized) {                                                                     \
        struct NedCoor_i ned_coor;                                                                  \
        struct NedCoor_i ned_vel;                                                                   \
        if(!autopilot_in_flight) {                                                                  \
          struct Int32Vect2 zero_vector;                                                            \
          INT_VECT2_ZERO(zero_vector);                                                              \
          gh_set_ref(zero_vector, zero_vector, zero_vector);                                        \
          INT_VECT2_ZERO(guidance_h_pos_ref);                                                       \
          INT_VECT2_ZERO(guidance_h_speed_ref);                                                     \
          INT_VECT2_ZERO(guidance_h_accel_ref);                                                     \
          gv_set_ref(0, 0, 0);                                                                      \
          guidance_v_z_ref = 0;		     \
          guidance_v_zd_ref = 0;		     \
          guidance_v_zdd_ref = 0;		     \
        }                                                                                           \
        VECT3_ASSIGN(ned_coor, guidance_h_pos_ref.x * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM,    \
                               guidance_h_pos_ref.y * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM,    \
                               guidance_v_z_ref * INT32_POS_OF_CM_DEN / INT32_POS_OF_CM_NUM);       \
        VECT3_ASSIGN(ned_vel, guidance_h_speed_ref.x * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM, \
                              guidance_h_speed_ref.y * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM, \
                              guidance_v_zd_ref * INT32_SPEED_OF_CM_S_DEN / INT32_SPEED_OF_CM_S_NUM);   \
        ecef_of_ned_point_i(&gps.ecef_pos, &ins_impl.ltp_def, &ned_coor);                                \
        ecef_of_ned_vect_i(&gps.ecef_vel, &ins_impl.ltp_def, &ned_vel);                                  \
        gps.fix = GPS_FIX_3D;                                                                       \
        gps.last_fix_time = sys_time.nb_sec;                                                        \
        gps_available = TRUE;                                                                       \
      }                                                                                             \
      else {                                                                                        \
        struct Int32Vect2 zero_vector;                                                              \
        INT_VECT2_ZERO(zero_vector);                                                                \
        gh_set_ref(zero_vector, zero_vector, zero_vector);                                          \
        gv_set_ref(0, 0, 0);                                                                        \
      }                                                                                             \
      _sol_available_callback();                                                                    \
      gps_available = FALSE;                                                                        \
    }                                                                                               \
  }

#endif /* GPS_SIM_HITL_H */
