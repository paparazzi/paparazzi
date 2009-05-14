
#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart_hw.h"
#include "uart.h"
#include "usb_serial.h"

int main( void ) {    
  unsigned char inc;
    
  hw_init();
  sys_time_init();
  led_init();

#ifdef USE_UART0
  Uart0Init();
#else
#ifdef USE_UART1
  Uart1Init();
#else
#error no serial port defined
#endif
#endif
    
#ifdef USE_USB_SERIAL
  VCOM_init();
#endif
    
  int_enable();

#ifdef USE_UART0
  while(1) {
    if (Uart0ChAvailable() && VCOM_check_free_space(1)) {
      inc = Uart0Getch();
      VCOM_putchar(inc);
    }
    if (VCOM_check_available() && uart0_check_free_space(1)) {
      inc = VCOM_getchar();
      uart0_transmit(inc);
    }
  }
#else
  while(1) {
    if (Uart1ChAvailable() && VCOM_check_free_space(1)) {
      inc = Uart1Getch();
      VCOM_putchar(inc);
    }
    if (VCOM_check_available() && uart1_check_free_space(1)) {
      inc = VCOM_getchar();
      uart1_transmit(inc);
    }
  }
#endif
  
  return 0;
}
