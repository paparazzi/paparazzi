#include "wt_servo.h"

#include "LPC21xx.h"

#include "mcu_periph/sys_time.h"

uint16_t wt_servo_motor_power;

#define MY_NB_CLOCK_TIMER_PWM(time_us) cpu_ticks_of_usec(time_us)

void mb_servo_set_ns(uint32_t duration_ns);

void wt_servo_init(void)
{

  /* set P0.21 as PWM5 output */
  PINSEL1 |= (0X01 << 10);
  /* enable and select the type of PWM channel */
  PWMPCR |= PWMPCR_ENA5;
  /* set Match0 value (refresh rate) */
  PWMMR0 = MY_NB_CLOCK_TIMER_PWM(20000);
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH0;
  /* enable PWM timer in PWM mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;

}

#define MB_SERVO_MIN_PULSE_NS   1000000
#define MB_SERVO_RANGE_PULSE_NS 1000

void wt_servo_set(uint16_t val)
{

  uint32_t pulse_ns = MB_SERVO_MIN_PULSE_NS + val * MB_SERVO_RANGE_PULSE_NS;
  mb_servo_set_ns(pulse_ns);

}

void mb_servo_set_ns(uint32_t duration_ns)
{
  /* set Match5 value (pulse duration )*/
  PWMMR5 = cpu_ticks_of_nsec(duration_ns);
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH5;
}
