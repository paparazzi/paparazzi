#ifndef SPI_HW_H
#define SPI_HW_H

extern volatile uint8_t spi_idx_buf;

#define SpiInitBuf() { \
  spi_idx_buf = 0; \
  SPDR = spi_buffer_input[0]; \
  spi_message_received = FALSE; \
}

#ifdef FBW

#define SPI_PORT   PORTB
#define SPI_PIN    PINB
#define SPI_SS_PIN 2

#define SpiStart() SpiInitBuf()

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

/* Enable SPI, Master, clock fck/16, interrupt */ 
#define SpiStart() { \
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR0); \
  uint8_t foo; \
  if (bit_is_set(SPSR, SPIF)) \
    foo = SPDR; \
  SPCR |= _BV(SPIE); \
  SpiInitBuf(); \
}

#define SpiSelectSlave0() { \
  spi_cur_slave = SPI_SLAVE0; \
  ClearBit( SPI_SS0_PORT, SPI_SS0_PIN );\
}

#define SpiUnselectSlave0() { \
  spi_cur_slave = SPI_NONE; \
  SetBit( SPI_SS0_PORT, SPI_SS0_PIN );\
}

#define SpiSelectSlave1() { \
  spi_cur_slave = SPI_SLAVE1; \
  ClearBit( SPI_SS1_PORT, SPI_SS1_PIN );\
}

#define SpiUnselectSlave1() { \
  spi_cur_slave = SPI_NONE; \
  SetBit( SPI_SS1_PORT, SPI_SS1_PIN );\
}

#endif /* AP */


#endif /* SPI_HW_H */
