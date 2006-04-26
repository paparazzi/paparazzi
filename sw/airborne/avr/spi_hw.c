#include "spi.h"


#include <inttypes.h>
#include <avr/io.h>

#if (__GNUC__ == 3)
#include <avr/signal.h>
#include <avr/crc16.h>
#else
#include <util/crc16.h>
#endif

#include <avr/interrupt.h>

#include "inter_mcu.h"

#ifdef FBW

#define IT_PORT PORTD
#define IT_DDR  DDRD
#define IT_PIN  7

#define SPI_DDR  DDRB
#define SPI_MOSI_PIN 3
#define SPI_MISO_PIN 4
#define SPI_SCK_PIN  5

volatile bool_t spi_was_interrupted = FALSE;

static volatile uint8_t idx_buf = 0;
static volatile uint16_t crc_in, crc_out;

void spi_init(void) {
  from_fbw.from_fbw.status = 0;
  from_fbw.from_fbw.nb_err = 0;

  /* set it pin output */
  //  IT_DDR |= _BV(IT_PIN);

  /* set MISO pin output */
  SPI_DDR |= _BV(SPI_MISO_PIN);
  /* enable SPI, slave, MSB first, sck idle low */
  SPCR = _BV(SPE);
  /* enable interrupt */
  SPCR |= _BV(SPIE);
}

void spi_reset(void) {
  idx_buf = 0;
  crc_in = CRC_INIT;
  crc_out = CRC_INIT;

  uint8_t first_byte = ((uint8_t*)&from_fbw.from_fbw)[0];
  crc_out = CrcUpdate(crc_out, first_byte);
  SPDR = first_byte;

  from_ap_receive_valid = FALSE;
}


SIGNAL(SIG_SPI) {
  static uint8_t tmp, crc_in1;
  
  idx_buf++;

  spi_was_interrupted = TRUE;

  if (idx_buf > FRAME_LENGTH)
    return;
  /* we have sent/received a complete frame */
  if (idx_buf == FRAME_LENGTH) {
    /* read second byte of crc from receive register */
    tmp = SPDR;
    /* notify valid frame  */
    if (crc_in1 == Crc1(crc_in) && tmp == Crc2(crc_in))
      from_ap_receive_valid = TRUE;
    else
      from_fbw.from_fbw.nb_err++;
    return;
  }

  if (idx_buf == FRAME_LENGTH - 1) {
    /* send the second byte of the crc_out */
    tmp = Crc2(crc_out);
    SPDR = tmp;
    /* get the first byte of the crc_in */
    crc_in1 = SPDR;
    return;
  } 

  /* we are sending/receiving payload       */
  if (idx_buf < FRAME_LENGTH - 2) {
    /* place new payload byte in send register */
    tmp = ((uint8_t*)&from_fbw.from_fbw)[idx_buf];
    SPDR = tmp;
    crc_out = CrcUpdate(crc_out, tmp);
  } 
  /* we are done sending the payload */
  else { // idx_buf == FRAME_LENGTH - 2
    /* place first byte of crc_out */
    tmp = Crc1(crc_out);
    SPDR = tmp;
  }
  
  /* read the byte from receive register */
  tmp = SPDR;
  ((uint8_t*)&from_ap)[idx_buf-1] = tmp;
  crc_in = CrcUpdate(crc_in, tmp);
}

#endif /** FBW */



#ifdef AP

#include "autopilot.h"
#include "link_mcu_ap.h"
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




#endif
