#include "mb_servo.h"

#include "mcu_periph/sys_time.h"
#define MY_NB_CLOCK_TIMER_PWM(time_us) cpu_ticks_of_usec(time_us)

uint32_t mb_servo_max_pulse_ns, mb_servo_min_pulse_ns;

void mb_servo_set_ns(uint32_t duration_ns);

void mb_servo_init(void)
{
  /* set P0.21 as PWM5 output */
  PINSEL1 |= (0X01 << 10);
  /* enable and select the type of PWM channel */
  PWMPCR |= PWMPCR_ENA5;
  /* set Match0 value (refresh rate) */
  PWMMR0 = MY_NB_CLOCK_TIMER_PWM(5000);
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH0;
  /* enable PWM timer in PWM mode */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE;
  mb_servo_min_pulse_ns = MIN_SERVO_NS;
  mb_servo_max_pulse_ns = MAX_SERVO_NS;
}

void mb_servo_set_range(uint32_t min_pulse_ns, uint32_t max_pulse_ns)
{
  mb_servo_min_pulse_ns = min_pulse_ns;
  mb_servo_max_pulse_ns = max_pulse_ns;
}

void mb_servo_set_us(uint32_t duration_us)
{
  /* set Match5 value (pulse duration )*/
  PWMMR5 = MY_NB_CLOCK_TIMER_PWM(duration_us);
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH5;
}

void mb_servo_set_ns(uint32_t duration_ns)
{
  /* set Match5 value (pulse duration )*/
  PWMMR5 = cpu_ticks_of_nsec(duration_ns);
  /* commit PWMMRx changes */
  PWMLER = PWMLER_LATCH5;
}

/* normalized throttle between 0. and 1. */
void mb_servo_set(float throttle)
{
  uint32_t range = mb_servo_max_pulse_ns - mb_servo_min_pulse_ns;
  uint32_t pulse_ns = mb_servo_min_pulse_ns + throttle * (range);
  mb_servo_set_ns(pulse_ns);
}

#define FOO_DELAY() {       \
    uint32_t foo = 0;       \
    while (foo<10000000) foo++;     \
  }


/* arm the brushless controller */

void mb_servo_arm(void)
{
  mb_servo_set(0.);
  FOO_DELAY();
  mb_servo_set(1.);
  FOO_DELAY();
  mb_servo_set(0.);
}

