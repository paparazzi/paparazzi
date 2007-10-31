#include "booz_autopilot.h"

#include "radio_control.h"
#include "commands.h"
#include "booz_control.h"
#include "booz_nav.h"

uint8_t booz_autopilot_mode;

void booz_autopilot_init(void) {
  booz_autopilot_mode = BOOZ_AP_MODE_FAILSAFE;
}

void booz_autopilot_periodic_task(void) {

  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_FAILSAFE:
  case BOOZ_AP_MODE_KILL:
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
  case BOOZ_AP_MODE_NAV:
    booz_nav_run();
    SetCommands(booz_control_commands);
    break;
  }

}


void booz_autopilot_on_rc_event(void) {
  /* I think this should be hidden in rc code */
  /* the ap gets a mode everytime - the rc filters it */
  if (rc_values_contains_avg_channels) {
    booz_autopilot_mode = BOOZ_AP_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    rc_values_contains_avg_channels = FALSE;
  }
  switch (booz_autopilot_mode) {
  case BOOZ_AP_MODE_RATE:
    booz_control_rate_read_setpoints_from_rc();
    break;
  case BOOZ_AP_MODE_ATTITUDE:
    booz_control_attitude_read_setpoints_from_rc();
    break;
  case BOOZ_AP_MODE_NAV:
    booz_nav_read_setpoints_from_rc();
    break;
  }
}
