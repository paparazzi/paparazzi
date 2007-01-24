#include "ant_spi.h"

#include <avr/io.h>
#include "std.h"


/*********************************Flush SPI*****************************************/


void flush_SPI( void )
{
  if (bit_is_set(SPSR, SPIF))
    {
      uint8_t foo __attribute__ ((unused)) = SPDR;
    }
}


/*****************************Initialisation SPI************************************/

#define DDR_SPI  DDRB
#define PORT_SPI PORTB
#define PIN_SS   0
#define PIN_SCK  1
#define PIN_MOSI 2
#define PIN_SYNC 4

void SPI_select_slave ( void )
{
    ClearBit(PORT_SPI, PIN_SS);
}

/***********************************************************************************/

void  SPI_unselect_slave ( void )
{
    SetBit(PORT_SPI, PIN_SS);
}

/***********************************************************************************/

void  SPI_master_init( void )
{
  /* Set SS, MOSI and SCK output, all others input */
  DDR_SPI |= _BV(PIN_SS) | _BV(PIN_SCK) | _BV(PIN_MOSI);
  /* unselect slave */
  SPI_unselect_slave();
  /* Enable SPI, Master, MSB first, clock idle low, sample on leading edge, clock rate fck/128 */
  SPCR = ( _BV(SPE)| _BV(MSTR) | _BV(SPR1)  |  _BV(SPR0));
}

/***********************************************************************************/

void  SPI_start( void )
{
  SPI_select_slave();
  if (bit_is_set(SPSR, SPIF)) {
    uint8_t foo __attribute__ ((unused)) = SPDR;
  }
  /* enable interrupt */
  SetBit(SPCR,SPIE);
}

/***********************************************************************************/

void SPI_stop(void)
{
  SPI_unselect_slave ();
  /* disable interrupt */
  ClearBit(SPCR,SPIE);
}
