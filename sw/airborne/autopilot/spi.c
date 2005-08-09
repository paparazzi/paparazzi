#include <inttypes.h>
#include <avr/io.h>
#include <avr/signal.h>
#include <avr/interrupt.h>


#include "spi.h"
#include "autopilot.h"
#include "link_fbw.h"
#include "ad7714.h"

volatile uint8_t spi_cur_slave;
uint8_t spi_nb_ovrn;

void spi_init( void) {
  /* Set MOSI and SCK output, all others input */ 
  SPI_DDR |= _BV(SPI_MOSI_PIN)| _BV(SPI_SCK_PIN); 

  /* enable pull up for miso */
  //  SPI_PORT |= _BV(SPI_MISO_PIN);

  /* Set SS0 output */
  sbi( SPI_SS0_DDR, SPI_SS0_PIN);
  /* SS0 idles high (don't select slave yet)*/
  SPI_UNSELECT_SLAVE0();

  /* Set SS1 output */
  sbi( SPI_SS1_DDR, SPI_SS1_PIN);
  /* SS1 idles high (don't select slave yet)*/
  SPI_UNSELECT_SLAVE1();
  
  spi_cur_slave = SPI_NONE;
}


SIGNAL(SIG_SPI) {
  if (spi_cur_slave == SPI_SLAVE0)
    link_fbw_on_spi_it();
  else
    fatal_error_nb++;
}
