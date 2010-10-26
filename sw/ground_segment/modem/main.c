#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>
#include <math.h>


#include "timer.h"
#include "soft_uart.h"
#include "adc.h"
#include "uart.h"
#include "link_tmtc.h"

#define FALSE 0
#define TRUE (!FALSE)

static uint16_t cputime = 0; // seconds

#define INPUT_BUF_LEN 10
static uint8_t input_buf[INPUT_BUF_LEN];
static uint8_t input_buf_idx = 0;

static uint16_t saved_valim;

inline void periodic_task( void ) { // 15 Hz
  static uint8_t _1Hz = 0; 
  _1Hz++;
  if (_1Hz>=15) _1Hz=0;
  
  if (!_1Hz) {
    uint8_t cd_status = bit_is_set(SOFT_UART_CD_PIN, SOFT_UART_CD);
    cputime++;
    LINK_TMTC_SEND_CD(cd_status);
    LINK_TMTC_SEND_VALIM(&saved_valim);
    LINK_TMTC_SEND_DEBUG();
  }
}

int main( void ) {
  /* init peripherals */
  timer_init(); 
  uart_init();
  soft_uart_init();
  adc_init();
  sei();

  /*  enter mainloop */
  while( 1 ) {
    if(timer_periodic())
      periodic_task();
    if (soft_uart_error) {
      LINK_TMTC_SEND_ERROR(soft_uart_error);
      soft_uart_error = 0;
    }
    if (soft_uart_got_byte) {
      input_buf[input_buf_idx] = soft_uart_byte;
      input_buf_idx++;
      if (input_buf_idx >= INPUT_BUF_LEN) {
	LINK_TMTC_SEND_DATA(input_buf, input_buf_idx); 
	input_buf_idx = 0;
      }
      soft_uart_got_byte = FALSE;
    }
    if (adc_got_val) {
      saved_valim = adc_alim;
      adc_got_val = FALSE;
    }
  } 
  return 0;
}
