#include "std.h"
#include "mcu.h"
#include "mcu_periph/sys_time.h"
#include "led.h"
#include "mcu_periph/usb_serial.h"
#include "mcu_periph/uart.h"
#include "mcu_arch.h"

#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

#include "armVIC.h"

#ifndef NB_STEP
#define NB_STEP 256
#endif

static inline void main_init(void);
static inline void main_periodic(void);
static inline void main_event(void);

static inline void main_init_tacho(void);
uint32_t lp_pulse;
uint32_t nb_pulse = 0;
static float omega_rad;


int main(void)
{
  main_init();
  while (1) {
    if (sys_time_check_and_ack_timer(0)) {
      main_periodic();
    }
    main_event();
  }
  return 0;
}

static inline void main_init(void)
{
  mcu_init();
  sys_time_register_timer(1. / PERIODIC_FREQUENCY, NULL);
  main_init_tacho();
  mcu_int_enable();
}

static inline void main_periodic(void)
{

  RunOnceEvery(50, {
    const float tach_to_rpm = 15000000.*2 * M_PI / (float)NB_STEP;
    omega_rad = tach_to_rpm / lp_pulse;
    DOWNLINK_SEND_IMU_TURNTABLE(DefaultChannel, DefaultDevice, &omega_rad);
  }
  //      float foo = nb_pulse;
  //      DOWNLINK_SEND_IMU_TURNTABLE(DefaultChannel, DefaultDevice, &foo);}
              );
  RunOnceEvery(100, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice,  16, MD5SUM);});

}


static inline void main_event(void)
{

}




/* INPUT CAPTURE CAP0.0 on P0.22*/
#define TT_TACHO_PINSEL     PINSEL1
#define TT_TACHO_PINSEL_VAL 0x02
#define TT_TACHO_PINSEL_BIT 12

static inline void main_init_tacho(void)
{
  /* select pin for capture */
  TT_TACHO_PINSEL |= TT_TACHO_PINSEL_VAL << TT_TACHO_PINSEL_BIT;
  /* enable capture 0.2 on falling edge + trigger interrupt */
  T0CCR |= TCCR_CR0_F | TCCR_CR0_I;
}
