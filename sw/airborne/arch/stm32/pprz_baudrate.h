#ifndef __PPRZ_BAUDRATE_H
#define __PPRZ_BAUDRATE_H

#include BOARD_CONFIG

#ifdef USE_OPENCM3
void usart_set_baudrate(void *usart, uint32_t baud);
#define pprz_usart_set_baudrate(x, y) usart_set_baudrate(x, y)
#else
#define pprz_usart_set_baudrate(x, y) do { } while(0);
#endif

#endif
