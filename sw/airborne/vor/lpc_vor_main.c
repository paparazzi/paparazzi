#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

#include "lpc_vor_convertions.h"
#include "vor_int_demod.h"


static inline void main_init( void );


int main( void ) {
  main_init();
  while(1) {
    //    if (vor_adc_sample_available) {
      LED_OFF(1);

      //      uint32_t foo = 0;
      //      while (foo < (2<<10)) foo++;

      //      vor_adc_sample_available = FALSE;

      vor_int_demod_run (vor_adc_sample);

      //      VorDacSet(vor_adc_sample);
   
      LED_ON(1);
      //    }
      uint32_t bar = 0;
      while (bar < 500) bar++;

  }

  return 0;
}

static inline void main_init( void ) {
  hw_init();
  //  sys_time_init();
  led_init();
  vor_int_demod_init();
  //  VorDacInit();
  //  vor_adc_init();
  //  int_enable();
}




