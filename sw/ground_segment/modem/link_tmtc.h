#ifndef LINK_TMTC_H
#define LINK_TMTC_H

#define STX         0x02
#define ETX         0x03

#define MSG_DATA    0
#define MSG_ERROR   1
#define MSG_CD      2
#define MSG_DEBUG   3
#define MSG_VALIM   4



#define LINK_TMTC_SEND_DATA(data, _len) { \
  uint8_t checksum = 0; \
  const uint8_t real_len = 2+_len; \
  uint8_t i; \
  uart_putc(STX); \
  uart_putc(real_len); \
  checksum^=real_len; \
  uart_putc(MSG_DATA); \
  checksum^=MSG_DATA; \
  for (i=0; i<_len; i++) { \
    uart_putc(data[i]); \
    checksum^=data[i]; \
  } \
  uart_putc(checksum); \
  uart_putc(ETX); \
}

#define LINK_TMTC_SEND_ERROR(error) { \
  uint8_t checksum = 0; \
  const uint8_t real_len = 2+1; \
  uart_putc(STX); \
  uart_putc(real_len); \
  checksum^=real_len; \
  uart_putc(MSG_ERROR); \
  checksum^=MSG_ERROR; \
  uart_putc(error); \
  checksum^=error; \
  uart_putc(checksum); \
  uart_putc(ETX); \
}


#define LINK_TMTC_SEND_CD(cd) { \
  uint8_t checksum = 0; \
  const uint8_t real_len = 2+1; \
  uart_putc(STX); \
  uart_putc(real_len); \
  checksum^=real_len; \
  uart_putc(MSG_CD); \
  checksum^=MSG_CD; \
  uart_putc(cd); \
  checksum^=cd; \
  uart_putc(checksum); \
  uart_putc(ETX); \
}

#define LINK_TMTC_SEND_DEBUG() { \
  uint8_t checksum = 0; \
  const uint8_t real_len = 2+1; \
  uart_putc(STX); \
  uart_putc(real_len); \
  checksum^=real_len; \
  uart_putc(MSG_DEBUG); \
  checksum^=MSG_DEBUG; \
  uart_putc(uart_nb_ovrrun); \
  checksum^=uart_nb_ovrrun; \
  uart_putc(checksum); \
  uart_putc(ETX); \
}

#define LINK_TMTC_SEND_VALIM(_valim) { \
  uint8_t checksum = 0; \
  const uint8_t real_len = 2+2; \
  uart_putc(STX); \
  uart_putc(real_len); \
  checksum^=real_len; \
  uart_putc(MSG_VALIM); \
  checksum^=MSG_VALIM; \
  uart_putc(*(uint8_t*)(_valim)); \
  checksum^= *(uint8_t*)(_valim); \
  uart_putc(* ((uint8_t*)(_valim) + 1)); \
  checksum^= *((uint8_t*)(_valim) + 1); \
  uart_putc(checksum); \
  uart_putc(ETX); \
}


#endif
