#include "spi.h"

uint8_t* spi_buffer_input;
uint8_t* spi_buffer_output;
uint8_t spi_buffer_length;
volatile bool_t spi_message_received;

#ifdef AP

volatile uint8_t spi_cur_slave;
uint8_t spi_nb_ovrn;

#endif /* AP */
