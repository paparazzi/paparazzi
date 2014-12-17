#ifndef ArduIMU_H
#define ArduIMU_H

#include <inttypes.h>

#define NB_DATA 6

extern float ArduIMU_data[NB_DATA];

extern float ins_roll_neutral;
extern float ins_pitch_neutral;

//mixer
extern float pitch_of_throttle_gain;
extern float throttle_slew;

void ArduIMU_init(void);
void ArduIMU_periodic(void);
void ArduIMU_periodicGPS(void);
void IMU_Daten_verarbeiten(void);

#endif // ArduIMU_H
