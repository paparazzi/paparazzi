#ifndef SPI_H
#define SPI_H

#include "std.h"

#include CONFIG
#include "spi_hw.h"


#ifdef FBW
extern volatile bool_t spi_was_interrupted;

void spi_init(void);
void spi_reset(void);

#endif

#ifdef AP
#define SPI_NONE 0
#define SPI_SLAVE0 1
#define SPI_SLAVE1 2

extern volatile uint8_t spi_cur_slave;
extern uint8_t spi_nb_ovrn;
void spi_init( void);

#endif

#endif /* SPI_H */


