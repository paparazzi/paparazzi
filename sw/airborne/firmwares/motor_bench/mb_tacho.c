#include "mb_tacho.h"

#include "LPC21xx.h"

#include "mcu.h"

volatile uint32_t mb_tacho_duration;
volatile uint8_t got_one_pulse;
volatile float mb_tacho_averaged;
volatile uint16_t  mb_tacho_nb_pulse;

/* INPUT CAPTURE CAP0.0 on P0.22*/
#define MB_TACHO_PINSEL     PINSEL1
#define MB_TACHO_PINSEL_VAL 0x02
#define MB_TACHO_PINSEL_BIT 12

#define MB_TACHO_NB_SLOT 65
//#define MB_TACHO_NB_SLOT 36

void mb_tacho_init(void)
{
  /* select pin for capture */
  MB_TACHO_PINSEL |= MB_TACHO_PINSEL_VAL << MB_TACHO_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR0_F | TCCR_CR0_I;
}

uint32_t mb_tacho_get_duration(void)
{
  int_disable();
  uint32_t my_duration = 0;
  if (got_one_pulse) {
    my_duration = mb_tacho_duration;
  }
  got_one_pulse = false;
  mcu_int_enable();
  return my_duration;
}

float mb_tacho_get_averaged(void)
{

  int_disable();
  float ret;
  float tacho;
  const float tach_to_rpm = 15000000.*60. / (float)MB_TACHO_NB_SLOT;
  if (mb_tacho_nb_pulse) {
    tacho = mb_tacho_averaged / (float)mb_tacho_nb_pulse ;
  } else {
    tacho = 0.;
  }

  if (tacho == 0) {
    ret = 0;
  } else {
    ret = tach_to_rpm / tacho;
  }

  mb_tacho_averaged = 0.;
  mb_tacho_nb_pulse = 0;
  mcu_int_enable();

  return ret;

}
