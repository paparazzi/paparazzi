#include <avr/io.h>

#define MODEM_CHECK_RUNNING() { \
  if (!(EIMSK & _BV(MODEM_CLK_INT))) { \
    MODEM_LOAD_NEXT_BYTE() \
    sbi(EIFR, INTF0); \
    sbi(EIMSK, MODEM_CLK_INT); \
  } \
}

