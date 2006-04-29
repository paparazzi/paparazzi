#include "spi.h"

uint8_t* spi_buffer_input;
uint8_t* spi_buffer_output;
uint8_t spi_buffer_length;
volatile bool_t spi_message_received;
