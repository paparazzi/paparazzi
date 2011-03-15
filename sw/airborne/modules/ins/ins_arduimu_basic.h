#ifndef ArduIMU_H
#define ArduIMU_H

#include <inttypes.h>
#include "math/pprz_algebra_float.h"

extern struct FloatEulers arduimu_eulers;
extern struct FloatRates arduimu_rates;
extern struct FloatVect3 arduimu_accel;

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

void ArduIMU_init( void );
void ArduIMU_periodic( void );
void ArduIMU_periodicGPS( void );
void ArduIMU_event( void );

#endif // ArduIMU_H
