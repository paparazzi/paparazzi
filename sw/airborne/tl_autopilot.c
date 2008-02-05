#include "tl_autopilot.h"

#include "radio_control.h"
#include "commands.h"
#include "tl_control.h"
#include "tl_nav.h"

uint8_t tl_autopilot_mode;

void tl_autopilot_init(void) {
  tl_autopilot_mode = TL_AP_MODE_FAILSAFE;
}

void tl_autopilot_periodic_task(void) {

  switch (tl_autopilot_mode) {
  case TL_AP_MODE_FAILSAFE:
  case TL_AP_MODE_KILL:
    SetCommands(commands_failsafe);
    break;
  case TL_AP_MODE_RATE:
    tl_control_rate_run();
    SetCommands(tl_control_commands);
    break;
  case TL_AP_MODE_ATTITUDE:
    tl_control_attitude_run();
    tl_control_agl_run();
    SetCommands(tl_control_commands);
    break;
  case TL_AP_MODE_SPEED:
    tl_control_speed_run();
    tl_control_attitude_run();
    SetCommands(tl_control_commands);
    break;
  case TL_AP_MODE_NAV:
    /* 4Hz */
    //    RunOnceEvery(15, { common_nav_periodic_task_4Hz(); tl_nav_periodic_task(); });
    tl_control_attitude_run();
    SetCommands(tl_control_commands);
    break;
  }
}


void tl_autopilot_on_rc_event(void) {
  /* I think this should be hidden in rc code */
  /* the ap gets a mode everytime - the rc filters it */
  if (rc_values_contains_avg_channels) {
    tl_autopilot_mode = TL_AP_MODE_OF_PPRZ(rc_values[RADIO_MODE]);
    rc_values_contains_avg_channels = FALSE;
  }
  switch (tl_autopilot_mode) {
  case TL_AP_MODE_RATE:
    tl_control_rate_read_setpoints_from_rc();
    break;
  case TL_AP_MODE_ATTITUDE:
    tl_control_attitude_read_setpoints_from_rc();
    break;
  case TL_AP_MODE_SPEED:
    tl_control_speed_read_setpoints_from_rc();
    break;
  case TL_AP_MODE_NAV:
    tl_control_nav_read_setpoints_from_rc();
    break;
  }
}
