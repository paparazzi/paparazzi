#ifndef BOOZ2_INS_H
#define BOOZ2_INS_H

#include "std.h"
#include "booz_geometry_int.h"
#include "pprz_geodetic_int.h"

/* gps transformed to LTP-NED  */
extern struct LtpDef_i  booz_ins_ltp_def;
extern          bool_t  booz_ins_ltp_initialised;
extern struct NedCoor_i booz_ins_gps_pos_cm_ned;
extern struct NedCoor_i booz_ins_gps_speed_cm_s_ned;

/* barometer                   */
#ifdef USE_VFF
extern int32_t  booz_ins_baro_alt;
extern int32_t  booz_ins_qfe;
extern bool_t   booz_ins_baro_initialised;
#endif

/* output                      */
extern struct NedCoor_i booz_ins_ltp_pos;
extern struct NedCoor_i booz_ins_ltp_speed;
extern struct NedCoor_i booz_ins_ltp_accel;

extern struct Pprz_int32_lla booz_ins_position_init_lla;
extern struct Pprz_int32_lla booz_ins_position_lla;
extern struct Pprz_int32_vect3 booz_ins_position;
extern struct Pprz_int32_vect3 booz_ins_speed_earth;
extern struct Pprz_int32_vect3 booz_ins_accel_earth;



extern void booz_ins_init( void );
extern void booz_ins_propagate( void );
extern void booz_ins_update_baro( void );
extern void booz_ins_update_gps( void );


#endif /* BOOZ2_INS_H */
