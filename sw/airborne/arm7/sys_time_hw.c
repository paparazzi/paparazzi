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

#ifdef RADIO_CONTROL
#include "ppm.h"
#else
#define PPM_IT 0x00
#endif

#if defined USE_RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
#include RADIO_CONTROL_TYPE_H
#else
#define RADIO_CONTROL_PPM_IT 0x00
#endif

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
#include "pwm_input.h"
#else
#define PWM_INPUT_IT 0x00
#endif

#ifdef USE_AMI601
#include "AMI601.h"
#else
#define AMI_601_IT 0x00
#endif


#define TIMER0_IT_MASK (ACTUATORS_IT         |\
                        PPM_IT               |\
                        RADIO_CONTROL_PPM_IT |\
                        MB_SCALE_IT          |\
                        MB_TACHO_IT          |\
                        PWM_INPUT_IT         |\
                        AMI_601_IT)

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

#ifdef RADIO_CONTROL
    if (T0IR&PPM_IT) {
      PPM_ISR();
      T0IR = PPM_IT;
    }
#endif
#if defined USE_RADIO_CONTROL && defined RADIO_CONTROL_TYPE_PPM
    if (T0IR&RADIO_CONTROL_PPM_IT) {
      RADIO_CONTROL_PPM_ISR();
      T0IR = RADIO_CONTROL_PPM_IT;
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
#ifdef USE_PWM_INPUT
    if (T0IR&PWM_INPUT_IT) {
      PWM_INPUT_ISR();
      T0IR = PWM_INPUT_IT; 
    }
#endif
#ifdef USE_AMI601
    if (T0IR&AMI_601_IT) {
      AMI_601_ISR();
      T0IR = AMI_601_IT; 
    }
#endif
  }
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
