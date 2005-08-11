#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/boot.h>
#include <avr/pgmspace.h>

#define BL_SIZE 1024
#define APP_END (FLASHEND - BL_SIZE)
#define PARTCODE  0x66
#define SIG_BYTE3 0x94
#define SIG_BYTE2 0x02
#define SIG_BYTE1 0x1E

void (*user_app)( void ) = 0x0000;

static void uart_init ( void ) {
  /* Baudrate is 38.4k */
  UBRR0H = 0;
  UBRR0L = 25;
  /* single speed */
  UCSR0A = 0;
  /* Enable receiver and transmitter */
  UCSR0B = _BV(RXEN) | _BV(TXEN);
  /* Set frame format: 8data, 1stop bit */
  UCSR0C = _BV(UCSZ1) | _BV(UCSZ0);
}

static void uart_putc( const uint8_t c ) {
  UDR0 = c;
  while( !bit_is_set( UCSR0A, TXC ) )
    ;
  UCSR0A |= _BV( TXC );
}

static uint8_t uart_getc( void ) {
  while( !bit_is_set( UCSR0A, RXC ) )
    ;
  return UDR0;
}


static void process_input ( uint8_t c ) {
  static uint16_t address;
  static uint16_t data;
  static uint8_t ldata;
  uint8_t output;

  switch (c) {
  case 'A': { /* Write address (MSB first) in word size increments*/
    uint16_t	high = uart_getc();
    address = ( (high<<8) | uart_getc() ) << 1;
    goto new_line;
  }
  case 'c': { /* Write program memory, low byte */
    ldata = uart_getc();
    goto new_line;
  }
  case 'C': { /* Write program memory, high byte */
    data = ldata | ( uart_getc() << 8 );
    boot_page_fill( data, address );
    address += 2;  
    goto new_line;   
  }
  case 'e': { /* Chip erase of application section */
    for( address=0; address < APP_END; address += SPM_PAGESIZE)
      boot_page_erase (address);
    goto new_line;
  }
  case 'a':   /* Query auto increment  */
  case 'n': { /* Query multiword-write */
    output = 'Y';
    goto out_char;
  }
  case 'M': { /* Write many words at once */
    uint8_t i;
    uint8_t len = uart_getc();
    uint16_t startaddr = address;
    for( i=0; i<len; i++ ) {
      ldata = uart_getc();
      data = ldata |(uart_getc()<<8);
      boot_page_fill( data, address );
      address += 2;  
    }
    uart_putc( len );
    uart_putc( (startaddr >> 8 ) & 0xFF );
    uart_putc( (startaddr >> 0 ) & 0xFF );
    uart_putc( (address >> 8 ) & 0xFF );
    uart_putc( (address >> 0 ) & 0xFF );
    goto new_line;
  }
  case 'm': { /* Write page: Perform page write */
    boot_page_write( address );
    boot_spm_busy_wait();
  }
  case 'P': { /* Enter programming mode */

  }
  case 'S': { /* Return software identifier */
    const uint8_t *s = "AVRBOOT";
    while( (c = *(s++) ) )
      uart_putc( c );
  }
  case 'V': { /* Return software version */
    uart_putc( '2' );
    output = '4';
    goto out_char;
  }
  case 'v': { /* Return hardware version */
    uart_putc( '1' );
    output = '0';
    goto out_char;
  }
  case 's': { /* Return signature byte */
    uart_putc( SIG_BYTE3 );
    uart_putc( SIG_BYTE2 );
    output = SIG_BYTE1;
    goto out_char;
  }
  case 'Z': { /* Start application (never returns) */
    boot_rww_enable ();
    user_app();
    break;
  }
  default: {
    output = '?';
    goto out_char;
  }
  }
  
 new_line:
  output = '\r';
 out_char:
  uart_putc( output );
  return;

}


int main ( void ) {
  cli();
  uart_init();
  
  while ( 1 ) {
    uint8_t c = uart_getc();
    process_input ( c );
    
  }
  return 0;
}
