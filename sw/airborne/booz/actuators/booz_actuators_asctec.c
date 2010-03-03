#include "booz_actuators.h"
#include "actuators/booz_actuators_asctec.h"

#include "booz2_commands.h"
#include "i2c.h"
#include "sys_time.h"

struct ActuatorsAsctec actuators_asctec; 

uint32_t actuators_delay_time;
bool_t   actuators_delay_done;

void actuators_init(void) {
  actuators_asctec.cmd = NONE;
  actuators_asctec.cur_addr = FRONT;
  actuators_asctec.new_addr = FRONT;
  actuators_asctec.i2c_done = TRUE;
  actuators_asctec.nb_err = 0;

#if defined BOOZ_START_DELAY && ! defined SITL
  actuators_delay_done = FALSE;
  SysTimeTimerStart(actuators_delay_time);
#else
  actuators_delay_done = TRUE;
  actuators_delay_time = 0;
#endif
}

void actuators_set(bool_t motors_on) {
#if defined BOOZ_START_DELAY && ! defined SITL
  if (!actuators_delay_done) {
    if (SysTimeTimer(actuators_delay_time) < SYS_TICS_OF_SEC(BOOZ_START_DELAY)) return;
    else actuators_delay_done = TRUE;
  }
#endif

  if (!actuators_asctec.i2c_done)
    actuators_asctec.nb_err++;

#ifdef KILL_MOTORS
  actuators_asctec.cmds[PITCH]  = 0;
  actuators_asctec.cmds[ROLL]   = 0;
  actuators_asctec.cmds[YAW]    = 0;
  actuators_asctec.cmds[THRUST] = 0;
#else /* ! KILL_MOTORS */
  actuators_asctec.cmds[PITCH]  = booz2_commands[COMMAND_PITCH]  + SUPERVISION_TRIM_E;
  actuators_asctec.cmds[ROLL]   = booz2_commands[COMMAND_ROLL]   + SUPERVISION_TRIM_A;
  actuators_asctec.cmds[YAW]    = booz2_commands[COMMAND_YAW]    + SUPERVISION_TRIM_R;
  actuators_asctec.cmds[THRUST] = booz2_commands[COMMAND_THRUST];
  Bound(actuators_asctec.cmds[PITCH],-100, 100);
  Bound(actuators_asctec.cmds[ROLL], -100, 100);
  Bound(actuators_asctec.cmds[YAW],  -100, 100);
  if (motors_on) {
    Bound(actuators_asctec.cmds[THRUST],  1, 200);
  }
  else
    actuators_asctec.cmds[THRUST] = 0;
#endif /* KILL_MOTORS  */

  switch (actuators_asctec.cmd) {
  case TEST:
    i2c0_buf[0] = 251;
    i2c0_buf[1] = actuators_asctec.cur_addr;
    i2c0_buf[2] = 0;
    i2c0_buf[3] = 231 + actuators_asctec.cur_addr;
    break;
  case REVERSE:
    i2c0_buf[0] = 254;
    i2c0_buf[1] = actuators_asctec.cur_addr;
    i2c0_buf[2] = 0;
    i2c0_buf[3] = 234 + actuators_asctec.cur_addr;
    break;
  case SET_ADDR:
    i2c0_buf[0] = 250;
    i2c0_buf[1] = actuators_asctec.cur_addr;
    i2c0_buf[2] = actuators_asctec.new_addr;
    i2c0_buf[3] = 230 + actuators_asctec.cur_addr + actuators_asctec.new_addr;
    actuators_asctec.cur_addr = actuators_asctec.new_addr;
    break;
  case NONE:
    i2c0_buf[0] = 100 -  actuators_asctec.cmds[PITCH];
    i2c0_buf[1] = 100 + actuators_asctec.cmds[ROLL];
    i2c0_buf[2] = 100 - actuators_asctec.cmds[YAW];
    i2c0_buf[3] = actuators_asctec.cmds[THRUST];
    break;
  }
  actuators_asctec.cmd = NONE;
  actuators_asctec.i2c_done = FALSE;
  i2c0_transmit(0x02, 4, &actuators_asctec.i2c_done);
}

