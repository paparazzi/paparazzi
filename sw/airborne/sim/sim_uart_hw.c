#include "uart_hw.h"

char stdinout_buffer[STDINOUT_BUFFER_SIZE];
uint8_t stdinout_rx_insert_idx = 0;
uint8_t stdinout_rx_extract_idx = 0;
