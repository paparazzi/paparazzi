#include "pt_ant_motors.h"

float pt_ant_motors_y_power;
float pt_ant_motors_z_power;

void pt_ant_motors_init(void) {
  /* configure direction pin as output */
  MOT_Y_DIR_DIR_REG |= 1 << MOT_Y_DIR_PIN;
  MOT_Z_DIR_DIR_REG |= 1 << MOT_Z_DIR_PIN;

  /* configure pins for PWM */
  MOT_Y_PWM_PINSEL |= MOT_Y_PWM_PINSEL_VAL << MOT_Y_PWM_PINSEL_BIT;
  MOT_Z_PWM_PINSEL |= MOT_Z_PWM_PINSEL_VAL << MOT_Z_PWM_PINSEL_BIT;
  /* set chopping frequency */
  PWMMR0 = MOT_CHOP_PERIOD;
  /* enable PWM outputs in single edge mode*/
  PWMPCR = MOT_Y_PWM_ENA | MOT_Z_PWM_ENA;
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH0;
  /* enable PWM timer in PWM mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;

  pt_ant_motors_SetYPower(0.);
  pt_ant_motors_SetZPower(0.);
}
