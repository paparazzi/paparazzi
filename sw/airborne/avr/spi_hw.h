#ifndef SPI_HW_H
#define SPI_HW_H

#ifdef FBW

#define SPI_PORT   PORTB
#define SPI_PIN    PINB
#define SPI_SS_PIN 2

#define SpiIsSelected() (bit_is_clear(SPI_PIN, SPI_SS_PIN))

#endif /* FBW */

#ifdef AP

#define SPI_SS0_PIN  0
#define SPI_SS0_PORT PORTB
#define SPI_SS0_DDR  DDRB
#define SPI_IT0_PIN  1
#define SPI_IT0_PORT PORTD
#define SPI_IT0_DDR  DDRD

#define SPI_SS1_PIN  7
#define SPI_SS1_PORT PORTE
#define SPI_SS1_DDR  DDRE
#define SPI_IT1_PIN  6
#define SPI_IT1_PORT PORTE
#define SPI_IT1_DDR  DDRE 

#define SPI_SCK_PIN  1
#define SPI_MOSI_PIN 2
#define SPI_MISO_PIN 3
#define SPI_PORT PORTB
#define SPI_DDR  DDRB

#define SPI_START(_SPCR_VAL) { \
  uint8_t foo; \
  SPCR = _SPCR_VAL; \
  if (bit_is_set(SPSR, SPIF)) \
    foo = SPDR; \
  SPCR |= _BV(SPIE); \
}

#define SPI_SELECT_SLAVE0() { \
  spi_cur_slave = SPI_SLAVE0; \
  cbi( SPI_SS0_PORT, SPI_SS0_PIN );\
}

#define SPI_UNSELECT_SLAVE0() { \
  spi_cur_slave = SPI_NONE; \
  sbi( SPI_SS0_PORT, SPI_SS0_PIN );\
}

#define SPI_SELECT_SLAVE1() { \
  spi_cur_slave = SPI_SLAVE1; \
  cbi( SPI_SS1_PORT, SPI_SS1_PIN );\
}

#define SPI_UNSELECT_SLAVE1() { \
  spi_cur_slave = SPI_NONE; \
  sbi( SPI_SS1_PORT, SPI_SS1_PIN );\
}

#define SPI_SEND(data) { \
  SPDR = data; \
}

#define SPI_STOP() { \
 cbi(SPCR,SPIE); \
 cbi(SPCR, SPE); \
}

#endif /* AP */


#endif /* SPI_HW_H */
