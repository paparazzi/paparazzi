#ifndef NPS_AUTOPILOT_H
#define NPS_AUTOPILOT_H

#include "generated/airframe.h"

#include "nps_radio_control.h"

/**
 * Number of commands sent to the FDM of NPS.
 * If MOTOR_MIXING_NB_MOTOR is defined (usually rotorcraft firmware)
 * we have that many commands (one per motor),
 * otherwise we default to the number of high level commands (COMMANDS_NB).
 */
#ifndef NPS_COMMANDS_NB
#if defined MOTOR_MIXING_NB_MOTOR
#define NPS_COMMANDS_NB MOTOR_MIXING_NB_MOTOR
#else
#ifdef NPS_ACTUATOR_NAMES
#define NPS_COMMANDS_NB COMMANDS_NB
#else
/* not using explicitly set NPS_ACTUATOR_NAMES -> throttle,roll,pitch,yaw commands */
#define NPS_COMMANDS_NB 4
#endif
#endif
#endif

struct NpsAutopilot {
  double commands[NPS_COMMANDS_NB];
  bool_t launch;
};

extern struct NpsAutopilot autopilot;

extern bool_t nps_bypass_ahrs;
extern bool_t nps_bypass_ins;
extern void sim_overwrite_ahrs(void);
extern void sim_overwrite_ins(void);

extern void nps_autopilot_init(enum NpsRadioControlType type, int num_script, char* js_dev);
extern void nps_autopilot_run_step(double time);
extern void nps_autopilot_run_systime_step(void);


#endif /* NPS_AUTOPILOT_H */

