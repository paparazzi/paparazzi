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

/*
 * Macros to generate i2cx_buf and i2cx_transmit from
 * ACTUATORS_ASCTEC_DEVICE
 *
 */
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
  actuators_asctec.i2c_trans.status = I2CTransSuccess;
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

  if (!actuators_asctec.i2c_trans.status == I2CTransSuccess)
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
    actuators_asctec.i2c_trans.buf[0] = 251;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 231 + actuators_asctec.cur_addr;
    break;
  case REVERSE:
    actuators_asctec.i2c_trans.buf[0] = 254;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = 0;
    actuators_asctec.i2c_trans.buf[3] = 234 + actuators_asctec.cur_addr;
    break;
  case SET_ADDR:
    actuators_asctec.i2c_trans.buf[0] = 250;
    actuators_asctec.i2c_trans.buf[1] = actuators_asctec.cur_addr;
    actuators_asctec.i2c_trans.buf[2] = actuators_asctec.new_addr;
    actuators_asctec.i2c_trans.buf[3] = 230 + actuators_asctec.cur_addr + actuators_asctec.new_addr;
    actuators_asctec.cur_addr = actuators_asctec.new_addr;
    break;
  case NONE:
    actuators_asctec.i2c_trans.buf[0] = 100 -  actuators_asctec.cmds[PITCH];
    actuators_asctec.i2c_trans.buf[1] = 100 + actuators_asctec.cmds[ROLL];
    actuators_asctec.i2c_trans.buf[2] = 100 - actuators_asctec.cmds[YAW];
    actuators_asctec.i2c_trans.buf[3] = actuators_asctec.cmds[THRUST];
    break;
  default:
    break;
  }
  actuators_asctec.cmd = NONE;

  i2c_submit(&i2c1,&actuators_asctec.i2c_trans);
  //  actuators_asctec.i2c_done = FALSE;
  //  DeviceTransmit(0x02, 4, &actuators_asctec.i2c_done);

}
#else /* ! ACTUATORS_ASCTEC_V2_PROTOCOL */
void actuators_set(bool_t motors_on) {
  if (!cpu_time_sec) return; // FIXME
  supervision_run(motors_on, FALSE, booz2_commands);
#ifdef KILL_MOTORS
  actuators_asctec.i2c_trans.buf[0] = 0;
  actuators_asctec.i2c_trans.buf[1] = 0;
  actuators_asctec.i2c_trans.buf[2] = 0;
  actuators_asctec.i2c_trans.buf[3] = 0;
  actuators_asctec.i2c_trans.buf[4] = 0xAA;
#else
  actuators_asctec.i2c_trans.buf[0] = supervision.commands[SERVO_FRONT];
  actuators_asctec.i2c_trans.buf[1] = supervision.commands[SERVO_BACK];
  actuators_asctec.i2c_trans.buf[2] = supervision.commands[SERVO_LEFT];
  actuators_asctec.i2c_trans.buf[3] = supervision.commands[SERVO_RIGHT];
  actuators_asctec.i2c_trans.buf[4] = 0xAA + actuators_asctec.i2c_trans.buf[0] + actuators_asctec.i2c_trans.buf[1] + 
                                             actuators_asctec.i2c_trans.buf[2] + actuators_asctec.i2c_trans.buf[3];
#endif

  i2c_submit(&i2c1,&actuators_asctec.i2c_trans);

  //  if (actuators_asctec.i2c_done) {
  //actuators_asctec.i2c_done = FALSE;
  //  DeviceTransmit(0x02, 5, &actuators_asctec.i2c_done);
  //  }
  
}
#endif /* ACTUATORS_ASCTEC_V2_PROTOCOL */
