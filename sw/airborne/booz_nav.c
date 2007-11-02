#include "booz_nav.h"

#include <math.h>

#include "booz_estimator.h"
#include "booz_control.h"
#include "radio_control.h"

#ifndef DISABLE_NAV
#include "booz_nav_hover.h"

#define BoozNavRunOnceEvery(_prescaler, _code) {	\
    static uint8_t prescaler = 0;			\
    prescaler++;					\
    if (prescaler > _prescaler) {			\
      prescaler = 0;					\
      _code;						\
    }							\
  }
#endif


void booz_nav_init(void) {
  booz_nav_hover_init();
}


void booz_nav_run(void) {
#ifndef DISABLE_NAV
  BoozNavRunOnceEvery( 10,						                        \
    {						                                                \
      booz_nav_hover_run();						                        \
      BoozControlAttitudeSetSetPoints(booz_nav_hover_phi_command, booz_nav_hover_theta_command, \
				     booz_nav_hover_psi_sp, booz_nav_hover_power_command);      \
   });
 
  booz_control_attitude_run();
#else
  /* on real ac, let nav kill all motors */
  booz_control_commands[COMMAND_P] = 0;
  booz_control_commands[COMMAND_Q] = 0;
  booz_control_commands[COMMAND_R] = 0;
  booz_control_commands[COMMAND_THROTTLE] = 0;
#endif

}

void booz_nav_read_setpoints_from_rc(void) {
#ifndef DISABLE_NAV
  booz_nav_hover_read_setpoints_from_rc();
#endif /* DISABLE_NAV */
}



