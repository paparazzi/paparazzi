//
// Created by ASUS on 07/12/2021.
//

#ifndef SYS_ID_WAVE_H
#define SYS_ID_WAVE_H
#endif // SYS_ID_WAVE_H

#include "paparazzi.h"


extern uint8_t wave_active;
extern uint8_t wave_axis;
extern pprz_t wave_amplitude;
extern float frequency_hz_;
extern float lag_rad_;

extern uint8_t wave_axis;

extern void sys_id_wave_init(void);
extern void sys_id_wave_run(void);

extern void sys_id_wave_axis_handler(uint8_t axis);
extern uint8_t sys_id_wave_running(void);

extern void sys_id_wave_frequency_hz_set(float frequency_hz_set);
extern void sys_id_wave_lag_rad_set(float lag_rad_set);

// handlers for changing in GCS variables
extern void sys_id_wave_activate_handler(uint8_t activate);
extern void sys_id_wave_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);




