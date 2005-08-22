#ifndef _3DMG_G
#define _3DMG_G

#include <inttypes.h>
#include "std.h"

void _3dmg_capture_gyro_bias ( void );
void _3dmg_set_continuous_mode ( void );
void _3dmg_capture_neutral ( void );

extern volatile bool_t _3dmg_data_ready;
extern int16_t         _3dmg_roll, _3dmg_pitch, _3dmg_yaw;
extern int16_t         _3dmg_roll_dot, _3dmg_pitch_dot, _3dmg_yaw_dot;

#endif /* _3DMG_G */
