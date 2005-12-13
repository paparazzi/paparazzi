#include "uart.h"

#include "traces.h"
#include <stdlib.h>

void uart0_print_hex(const uint8_t c) {
  const unsigned char hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                            '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
  unsigned char high = (c & 0xF0)>>4;
  unsigned char low  = c & 0x0F;
  uart0Putch(hex[high]);
  uart0Putch(hex[low]);
}

void uart0_print_hex_16(const uint16_t c) {
  unsigned char high = (c & 0xFF00)>>8;
  unsigned char low  = c & 0x00FF;
  uart0_print_hex(high);
  uart0_print_hex(low);
}

void uart0_print_hex_32(const uint32_t c) {
  uint16_t high = (c & 0xFFFF0000)>>16;
  uint16_t low  = c & 0x0000FFFF;
  uart0_print_hex_16(high);
  uart0_print_hex_16(low);
}

void uart0_print_dec_u32(const uint32_t c) {
  uint32_t c1 = c;
  uint32_t d = 100000;
  do {
    uint8_t e = c1 / d;
    uart0Putch(e%10+'0');
    d = d/10;
  } while (d>9);
  uart0Putch(c1%10+'0');
}

void uart0_print_dec_32(const int32_t c) {
  uint32_t c1 = c;
  if (c>=0) {
    uart0Putch(' ');
  }
  else {
    c1=-c;
    uart0Putch('-');
  }
  uint32_t d = 100000;
  do {
    uint8_t e = c1 / d;
    uart0Putch(e%10+'0');
    d = d/10;
  } while (d>9);
  uart0Putch(c1%10+'0');
}
