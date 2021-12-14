//
// Created by ASUS on 06/12/2021.
//

#ifndef SYS_ID_DOUBLET_H
#define SYS_ID_DOUBLET_H

#include "paparazzi.h"


extern uint8_t doublet_active;
extern pprz_t doublet_amplitude;

extern float doublet_length_s;
extern float doublet_extra_waiting_time_s;
// Index of doublet axis in ACTIVE_DOUBLET_AXES
extern uint8_t doublet_axis;

extern uint8_t doublet_mode_3211;

extern void sys_id_doublet_init(void);

// If doublet is running, update its values
extern void sys_id_doublet_run(void);

// Handlers for changing gcs variables
extern void sys_id_doublet_activate_handler(uint8_t activate); // Activate the doublet
extern void sys_id_doublet_axis_handler(uint8_t axis); // Check if new axis
extern void sys_id_doublet_mod3211_handler(uint8_t mode);
extern uint8_t sys_id_doublet_running(void);
// Add the current doublet values to the in_cmd values if motors_on is true
extern void sys_id_doublet_add_values(bool motors_on, bool override_on, pprz_t in_cmd[]);

#endif // SYS_ID_DOUBLET_H
