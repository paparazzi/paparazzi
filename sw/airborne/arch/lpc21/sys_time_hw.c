#include "armVIC.h"
#include "sys_time.h"

uint32_t cpu_time_ticks;
uint32_t last_periodic_event;

uint32_t sys_time_chrono_start; /* T0TC ticks */
uint32_t sys_time_chrono;       /* T0TC ticks */

#if defined ACTUATORS && ( defined SERVOS_4017 || defined SERVOS_4015_MAT || defined SERVOS_PPM_MAT)
#include ACTUATORS
#else
#define ACTUATORS_IT 0x00
#endif /* ACTUATORS */

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
#include "subsystems/radio_control.h"
#else
#define PPM_IT 0x00
#endif

//FIXME remove this and fix timer mask
#define RADIO_CONTROL_PPM_IT 0x00

#ifdef MB_SCALE
#include "mb_scale.h"
#else
#define MB_SCALE_IT 0x00
#endif

#ifdef MB_TACHO
#include "mb_tacho.h"
#else
#define MB_TACHO_IT 0x00
#endif

#ifdef USE_PWM_INPUT
#include "mcu_periph/pwm_input.h"
#endif
#ifndef USE_PWM_INPUT1
#define PWM_INPUT_IT1 0x00
#endif
#ifndef USE_PWM_INPUT2
#define PWM_INPUT_IT2 0x00
#endif

#ifdef USE_AMI601
#include "peripherals/ami601.h"
#else
#define AMI601_IT 0x00
#endif

#ifdef TRIGGER_EXT
#include "core/trigger_ext_hw.h"
#else
#define TRIGGER_IT 0x00
#endif

//FIXME: RADIO_CONTROL_PPM_IT not used anymore
#define TIMER0_IT_MASK (ACTUATORS_IT         |\
                        PPM_IT               |\
                        RADIO_CONTROL_PPM_IT |\
                        TRIGGER_IT           |\
                        MB_SCALE_IT          |\
                        MB_TACHO_IT          |\
                        PWM_INPUT_IT1        |\
                        PWM_INPUT_IT2        |\
                        AMI601_IT)

void TIMER0_ISR ( void ) {
  ISR_ENTRY();
  while (T0IR & TIMER0_IT_MASK) {

#if defined ACTUATORS && ( defined SERVOS_4017 || defined SERVOS_4015_MAT || defined SERVOS_PPM_MAT)
    if (T0IR&ACTUATORS_IT) {
#ifdef SERVOS_4017
      SERVOS_4017_ISR();
#endif
#ifdef SERVOS_4015_MAT
      Servos4015Mat_ISR();
#endif
#ifdef SERVOS_PPM_MAT
      ServosPPMMat_ISR();
#endif
      T0IR = ACTUATORS_IT;
    }
#endif /* ACTUATORS && (SERVOS_4017 || SERVOS_4015_MAT || SERVOS_PPM_MAT) */

#if defined RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
    if (T0IR&PPM_IT) {
      PPM_ISR();
      T0IR = PPM_IT;
    }
#endif
#ifdef TRIGGER_EXT
//#define TRIGGER_IT PPM_CRI
    if (T0IR&TRIGGER_IT) {
      TRIG_ISR();
      T0IR = TRIGGER_IT;
LED_TOGGLE(3);
    }
#endif
#ifdef MB_SCALE
    if (T0IR&MB_SCALE_IT) {
      MB_SCALE_ICP_ISR();
      T0IR = MB_SCALE_IT;
    }
#endif
#ifdef MB_TACHO
    if (T0IR&MB_TACHO_IT) {
      MB_TACHO_ISR();
      T0IR = MB_TACHO_IT;
    }
#endif
#ifdef USE_PWM_INPUT1
    if (T0IR&PWM_INPUT_IT1) {
      PWM_INPUT_ISR_1();
      T0IR = PWM_INPUT_IT1;
    }
#endif
#ifdef USE_PWM_INPUT2
    if (T0IR&PWM_INPUT_IT2) {
      PWM_INPUT_ISR_2();
      T0IR = PWM_INPUT_IT2;
    }
#endif
#ifdef USE_AMI601
    if (T0IR&AMI601_IT) {
      AMI601_ISR();
      T0IR = AMI601_IT;
    }
#endif
  }
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
