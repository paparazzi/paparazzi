#ifndef ANT_SPI_H
#define ANT_SPI_H

#include "std.h"

#include <avr/io.h>
#include "std.h"

void flush_SPI( void );
void SPI_select_slave ( void );
void SPI_unselect_slave ( void );
void SPI_master_init( void );
void SPI_start( void );
void SPI_stop( void );

#define SPI_transmit(c) {   SPDR = c; }
#define SPI_read() (SPDR)

#endif /* ANT_SPI_H */
