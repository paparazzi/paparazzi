#include "std.h"
#include "mcu.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "mcu_periph/usb_serial.h"
#include "mcu_periph/uart.h"
#include "mcu_arch.h"

#include "messages.h"
#include "downlink.h"

#include "armVIC.h"


static inline void main_init( void );
static inline void main_periodic( void );
static inline void main_event( void );

static inline void main_init_tacho(void);
static uint32_t lp_pulse;
static uint32_t nb_pulse = 0;
static float omega_rad;


int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic();
    main_event();
  }
  return 0;
}

static inline void main_init( void ) {
  mcu_init();
  sys_time_init();
  main_init_tacho();
  mcu_int_enable();
}

#define NB_STEP 256
static inline void main_periodic( void ) {

  RunOnceEvery(50, {
      const float tach_to_rpm = 15000000.*2*M_PI/(float)NB_STEP;
      omega_rad = tach_to_rpm / lp_pulse;
      DOWNLINK_SEND_IMU_TURNTABLE(DefaultChannel, &omega_rad);}
      //      float foo = nb_pulse;
      //      DOWNLINK_SEND_IMU_TURNTABLE(DefaultChannel, &foo);}
    );
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel,  16, MD5SUM);});

}


static inline void main_event( void ) {

}




/* INPUT CAPTURE CAP0.0 on P0.22*/
#define TT_TACHO_PINSEL     PINSEL1
#define TT_TACHO_PINSEL_VAL 0x02
#define TT_TACHO_PINSEL_BIT 12

static inline void main_init_tacho(void) {
  /* select pin for capture */
  TT_TACHO_PINSEL |= TT_TACHO_PINSEL_VAL << TT_TACHO_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR0_F | TCCR_CR0_I;
}


//
//  trimed version of arm7/sys_time_hw.c
//

uint32_t cpu_time_ticks;
uint32_t last_periodic_event;

uint32_t sys_time_chrono_start; /* T0TC ticks */
uint32_t sys_time_chrono;       /* T0TC ticks */


void TIMER0_ISR ( void ) {
  ISR_ENTRY();
  //  LED_TOGGLE(1);
  if (T0IR & TIR_CR0I) {
    static uint32_t pulse_last_t;
    uint32_t t_now = T0CR0;
    uint32_t diff = t_now - pulse_last_t;
    lp_pulse = (lp_pulse + diff)/2;
    pulse_last_t = t_now;
    nb_pulse++;
    //    got_one_pulse = TRUE;
    T0IR = TIR_CR0I;
  }
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
