#ifndef PRINT_H
#define PRINT_H

#define PrintString(out_fun, s) { \
  uint8_t i = 0;                  \
  while (s[i]) {                  \
    out_fun(s[i]);		  \
    i++;                          \
  }                               \
}

#define PrintHex(out_fun, c) {						\
    const uint8_t hex[16] = { '0', '1', '2', '3', '4', '5', '6', '7',   \
			      '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' }; \
    uint8_t high = (c & 0xF0)>>4;					\
    uint8_t low  = c & 0x0F;						\
    out_fun(hex[high]);							\
    out_fun(hex[low]);							\
  }									\
}

void PrintHex16 (out_fun, c ) {			\
  uint8_t high = (uint8_t)(c>>8);		\
  uint8_t low  = (uint8_t)(c);			\
  uart0_print_hex(high);			\
  uart0_print_hex(low);				\
}


#enfif PRINT_H

