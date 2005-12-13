

#define SPI_PORT   PORTB
#define SPI_PIN    PINB
#define SPI_SS_PIN 2

#define SpiIsSelected() (bit_is_clear(SPI_PIN, SPI_SS_PIN))
