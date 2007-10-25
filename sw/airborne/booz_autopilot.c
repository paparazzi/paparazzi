#include "booz_autopilot.h"

#include "commands.h"
#include "booz_control.h"

uint8_t booz_autopilot_mode;

void booz_autopilot_init(void) {
  booz_autopilot_mode = BOOZ_AP_MODE_FAILSAFE;
}

void booz_autopilot_periodic_task(void) {

  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_FAILSAFE:
    SetCommands(commands_failsafe);
    break;
  case BOOZ_AP_MODE_RATE:
    booz_control_rate_run();
    SetCommands(booz_control_commands);
    break;
  case BOOZ_AP_MODE_ATTITUDE:
    booz_control_attitude_run();
    SetCommands(booz_control_commands);
    break;
  }

}


void booz_autopilot_event_task(void) {

  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_RATE:
    booz_control_rate_compute_setpoints();
    break;
  case BOOZ_AP_MODE_ATTITUDE:
    booz_control_attitude_compute_setpoints();
    break;
  }

}
