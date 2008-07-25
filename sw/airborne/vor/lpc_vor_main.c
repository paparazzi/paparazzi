#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"
#include "print.h"
//#include "messages.h"
//#include "downlink.h"

#include "lpc_vor_convertions.h"
#include "vor_int_demod_decim.h"


static inline void main_init( void );
static inline void main_report( void );

int main( void ) {
  main_init();
  while(1) {
    if (vor_adc_sample_available) {
      int16_t off_sample = vor_adc_sample - 512;
      //      off_sample *= 2;

      vor_int_demod_run (off_sample);

      if (vid_qdr_available) {
      	vid_qdr_available = FALSE;
	main_report();
      }

      VorDacSet(vor_adc_sample);
      vor_adc_sample_available = FALSE;
    }
  }
  
  return 0;
}

/*//(vid_qdr*360.0)/(double)N_VAR_FM)49800*/
static inline void main_report( void ) {
  static uint8_t cnt = 0;
  static int32_t my_qdr;
  static uint8_t tmp;

  switch (cnt) {

  case 0:
    my_qdr = vid_qdr * 360 / 4980;
    if (my_qdr < 0) my_qdr+=3600;
    if (my_qdr > 3600) my_qdr-=3600;
    uart0_transmit('\r');
    break;
  case 1:
    tmp = my_qdr / 1000;
    my_qdr = my_qdr - 1000*tmp;
    uart0_transmit('0'+tmp);
    break;
  case 2:
    tmp = my_qdr / 100;
    my_qdr = my_qdr - 100*tmp;
    uart0_transmit('0'+tmp);
    break;
  case 3:
    tmp = my_qdr / 10;
    my_qdr = my_qdr - 10*tmp;
    uart0_transmit('0'+tmp);
    break;
  case 4:
    uart0_transmit('.');
    break;
  case 5:
    uart0_transmit('0'+my_qdr);
    break;
  case 6:
    uart0_transmit('\r');
    break;
  }

  cnt++;
  if (cnt >7) cnt = 0;
}




static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart0_init_tx();
  vor_int_demod_init();
  VorDacInit();
  vor_adc_init();
  int_enable();
}




