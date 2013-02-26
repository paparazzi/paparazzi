#ifndef MAX3100_H
#define MAX3100_H

#include <stdbool.h>
#include "std.h"
#include "mcu_periph/spi_arch.h"
#include "led.h"

/* Pin configuration for max3100 IRQ */
// #define MAX3100_IRQ_PIN 7
#if MAX3100_IRQ_PIN == 7
#define MAX3100_IRQ_PINSEL PINSEL0
#define MAX3100_IRQ_PINSEL_BIT 14
#define MAX3100_IRQ_PINSEL_VAL 0x3
#define MAX3100_IRQ_EINT 2
#define MAX3100_VIC_EINT VIC_EINT2
#elif MAX3100_IRQ_PIN == 16
#define MAX3100_IRQ_PINSEL PINSEL1
#define MAX3100_IRQ_PINSEL_BIT 0
#define MAX3100_IRQ_PINSEL_VAL 0x1
#define MAX3100_IRQ_EINT 0
#define MAX3100_VIC_EINT VIC_EINT0
#else
#error "Define MAX3100_IRQ_PIN"
#endif

#define MAX3100_SS_PORT 0
#define MAX3100_SS_PIN 20

#define MAX3100_IO__(port, reg) IO ## port ## reg
#define MAX3100_IO_(port, reg) MAX3100_IO__(port, reg)

#define MAX3100_SS_IOCLR MAX3100_IO_(MAX3100_SS_PORT, CLR)
#define MAX3100_SS_IODIR MAX3100_IO_(MAX3100_SS_PORT, DIR)
#define MAX3100_SS_IOSET MAX3100_IO_(MAX3100_SS_PORT, SET)

/** Max3100 protocol status */
#define MAX3100_STATUS_IDLE 0
#define MAX3100_STATUS_WRITING 1
#define MAX3100_STATUS_READING 2

extern volatile uint8_t max3100_status;
extern volatile bool max3100_data_available;
extern volatile bool max3100_transmit_buffer_empty; // Max3100 ready to receive data on SPI

/** I/O Buffers */
#define MAX3100_TX_BUF_LEN 256
#define MAX3100_RX_BUF_LEN 256

extern volatile uint8_t max3100_tx_insert_idx, max3100_tx_extract_idx;
extern volatile uint8_t max3100_rx_insert_idx, max3100_rx_extract_idx;

extern volatile uint8_t max3100_tx_buf[MAX3100_TX_BUF_LEN];
extern volatile uint8_t max3100_rx_buf[MAX3100_RX_BUF_LEN];

extern volatile uint8_t read_byte1, read_byte2;
extern bool read_bytes;

#define Max3100Select() {	\
    SetBit(MAX3100_SS_IOCLR, MAX3100_SS_PIN);	\
  }

#define Max3100Unselect() { \
    SetBit(MAX3100_SS_IOSET, MAX3100_SS_PIN);	\
  }

#define MAX3100_WRITE_CONF ((1U<<15) | (1U<<14))
#define MAX3100_READ_CONF ((0U<<15) | (1U<<14))
#define MAX3100_WRITE_DATA ((1U<<15) | (0U<<14))
#define MAX3100_READ_DATA ((0U<<15) | (0U<<14))

/* Datasheet page 12 */
#if MAX3100_FOSC == 1843200
#define MAX3100_B115200 0x0
#define MAX3100_B57600 0x1
#define MAX3100_B19200 0x9
#define MAX3100_B9600 0xA
#elif MAX3100_FOSC == 3686400
#define MAX3100_B9600 0xB
#else
#error "MAX3100_FOSC must be defined to 1843200 or 3686400"
#endif

#define MAX3100_BIT_NOT_RM (1U<<10)
#define MAX3100_BIT_NOT_TM (1U<<11)
#define MAX3100_BIT_NOT_FEN (1U<<13)
#define MAX3100_T_BIT 14
#define MAX3100_R_BIT 15

/** Like UART macros */
#define UART3100Init() {} /* Already initialized as a module */
#define UART3100CheckFreeSpace(_len) (((int16_t)max3100_tx_extract_idx - max3100_tx_insert_idx + MAX3100_TX_BUF_LEN - 1) % MAX3100_TX_BUF_LEN >= _len)

#define UART3100Transmit(_x) { max3100_putchar(_x); }
#define UART3100SendMessage() {}
#define UART3100Getch() ({\
   uint8_t ret = max3100_rx_buf[max3100_rx_extract_idx]; \
   max3100_rx_extract_idx++; /* Since size=256 */        \
   ret;                                                 \
})

#define UART3100ChAvailable() (max3100_rx_extract_idx != max3100_rx_insert_idx)

static inline void max3100_transmit(uint16_t data) {
  Max3100Select();
  SpiClearRti();
  SpiEnableRti();  /* enable rx fifo time out */
  SpiEnable();
  // SSPDR = data >> 8;
  // SSPDR = data & 0xff;
  SSPDR = data;
}

#define Max3100TransmitConf(_conf) max3100_transmit((_conf) | MAX3100_WRITE_CONF)
#define Max3100TransmitData(_data) max3100_transmit((_data) | MAX3100_WRITE_DATA)
#define Max3100ReadData() max3100_transmit(MAX3100_READ_DATA)


static inline void max3100_read_data(void) {
  Max3100ReadData();
  max3100_status = MAX3100_STATUS_READING;
}

static inline void max3100_flush( void ) {
  if (max3100_status == MAX3100_STATUS_IDLE
      && max3100_tx_extract_idx != max3100_tx_insert_idx
      && max3100_transmit_buffer_empty) {
    Max3100TransmitData(max3100_tx_buf[max3100_tx_extract_idx]);
    max3100_tx_extract_idx++; /* automatic overflow since len=256 */
    max3100_status = MAX3100_STATUS_WRITING;
    max3100_transmit_buffer_empty = false;
  }
}

/** Warning: No bufferring; SPI must be available */
static inline void max3100_putconfchar(char c) {
  Max3100TransmitConf(c);
  max3100_status = MAX3100_STATUS_WRITING;
}

static inline void max3100_putchar(char c) {
  max3100_tx_buf[max3100_tx_insert_idx] = c;
  max3100_tx_insert_idx++; /* automatic overflow since len=256 */
  /* flushed in the next event */
}

extern void max3100_init( void );
extern void max3100_debug( void );

static inline void max3100_event( void ) {
  if (read_bytes) {
    read_bytes = false;
    max3100_debug();
  }
  if (max3100_status == MAX3100_STATUS_IDLE) {
    if (max3100_data_available) {
      max3100_data_available = false;
      max3100_read_data();
    } else
      max3100_flush();
  }
}


#endif /* MAX3100_H */
