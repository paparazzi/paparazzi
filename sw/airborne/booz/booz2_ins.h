#ifndef BOOZ2_INS_H
#define BOOZ2_INS_H

#include "std.h"
#include "booz_geometry_int.h"

extern struct Pprz_int32_lla booz_ins_position_init_lla;

extern struct Pprz_int32_lla booz_ins_position_lla;
extern struct Pprz_int32_vect3 booz_ins_position;
extern struct Pprz_int32_vect3 booz_ins_speed_earth;
extern struct Pprz_int32_vect3 booz_ins_accel_earth;

#ifdef USE_VFD
extern int32_t booz_ins_g;
#endif
#ifdef USE_VFF
extern int32_t  booz_ins_baro_alt;
extern int32_t  booz_ins_qfe;
extern bool_t   booz_ins_baro_initialised;
#endif

extern void booz_ins_init( void );
extern void booz_ins_propagate( void );
extern void booz_ins_update_baro( void );
extern void booz_ins_update_gps( void );


#endif /* BOOZ2_INS_H */
