#include "csc_adc.h"
#include "csc_ap_link.h"
#include "csc_vane.h"

#include "LPC21xx.h"
#include "led.h"
#include "pwm_input.h"

#define PWM_INPUT_COUNTS_PER_REV 61358.
#define PWM_INPUT_MSG_PRESCALE 8
#define VANE_NB 2
const static float avg_factor = 0.5;

void csc_vane_init(void)
{

}

void csc_vane_periodic(void)
{
  static float angle[VANE_NB];
  static uint8_t prescale = 0;

  for (int i = 0; i < VANE_NB; i++) {
    angle[i] = avg_factor * angle[i] + (1. - avg_factor) * RadOfDeg(360. * pwm_input_duration[i] / PWM_INPUT_COUNTS_PER_REV);
  }

  if (prescale++ >= PWM_INPUT_MSG_PRESCALE) {
    prescale = 0;
    csc_ap_link_send_vane(angle);
  }
}

