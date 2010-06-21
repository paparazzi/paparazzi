#include "booz_actuators.h"
#include "actuators/booz_actuators_asctec.h"

#ifdef ACTUATORS_ASCTEC_V2_PROTOCOL
#include "actuators/booz_supervision.h"
#endif

#include "booz2_commands.h"
#include "i2c.h"
#include "sys_time.h"


#ifndef ACTUATORS_ASCTEC_DEVICE
#define ACTUATORS_ASCTEC_DEVICE i2c0
#endif

#define __Device(dev, _z) dev##_##_z 
#define  _Device(dev, _z) __Device(dev, _z)
#define  Device( _z) _Device(ACTUATORS_ASCTEC_DEVICE, _z)

#define   DeviceBuf                    Device(buf)
#define   DeviceTransmit(_x, _y, _z)   Device(transmit(_x, _y, _z))



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

#ifdef ACTUATORS_ASCTEC_V2_PROTOCOL
  supervision_init();
#endif

}

#ifndef ACTUATORS_ASCTEC_V2_PROTOCOL
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
    DeviceBuf[0] = 251;
    DeviceBuf[1] = actuators_asctec.cur_addr;
    DeviceBuf[2] = 0;
    DeviceBuf[3] = 231 + actuators_asctec.cur_addr;
    break;
  case REVERSE:
    DeviceBuf[0] = 254;
    DeviceBuf[1] = actuators_asctec.cur_addr;
    DeviceBuf[2] = 0;
    DeviceBuf[3] = 234 + actuators_asctec.cur_addr;
    break;
  case SET_ADDR:
    DeviceBuf[0] = 250;
    DeviceBuf[1] = actuators_asctec.cur_addr;
    DeviceBuf[2] = actuators_asctec.new_addr;
    DeviceBuf[3] = 230 + actuators_asctec.cur_addr + actuators_asctec.new_addr;
    actuators_asctec.cur_addr = actuators_asctec.new_addr;
    break;
  case NONE:
    DeviceBuf[0] = 100 -  actuators_asctec.cmds[PITCH];
    DeviceBuf[1] = 100 + actuators_asctec.cmds[ROLL];
    DeviceBuf[2] = 100 - actuators_asctec.cmds[YAW];
    DeviceBuf[3] = actuators_asctec.cmds[THRUST];
    break;
  }
  actuators_asctec.cmd = NONE;
  actuators_asctec.i2c_done = FALSE;
  DeviceTransmit(0x02, 4, &actuators_asctec.i2c_done);
}
#else /* ! ACTUATORS_ASCTEC_V2_PROTOCOL */
void actuators_set(bool_t motors_on) {
//  if (!cpu_time_sec) return; // FIXME
  if (cpu_time_sec < 2) return; // FIXME
  supervision_run(motors_on, FALSE, booz2_commands);
#ifdef KILL_MOTORS
  DeviceBuf[0] = 0;
  DeviceBuf[1] = 0;
  DeviceBuf[2] = 0;
  DeviceBuf[3] = 0;
  DeviceBuf[4] = 0xAA;
#else
  DeviceBuf[0] = supervision.commands[SERVO_FRONT];
  DeviceBuf[1] = supervision.commands[SERVO_BACK];
  DeviceBuf[2] = supervision.commands[SERVO_LEFT];
  DeviceBuf[3] = supervision.commands[SERVO_RIGHT];
  DeviceBuf[4] = 0xAA + DeviceBuf[0] + DeviceBuf[1] + DeviceBuf[2] + DeviceBuf[3];
#endif

  if (actuators_asctec.i2c_done) {
    actuators_asctec.i2c_done = FALSE;
    DeviceTransmit(0x02, 5, &actuators_asctec.i2c_done);
  }

}
#endif /* ACTUATORS_ASCTEC_V2_PROTOCOL */
