#ifndef MAX3100_H
#define MAX3100_H

#include <stdbool.h>
#include "std.h"
#include "spi_hw.h"

/* Pin configuration for max3100 IRQ */
#define MAX3100_IRQ_PIN 16
#define MAX3100_IRQ_PINSEL PINSEL1
#define MAX3100_IRQ_PINSEL_BIT 0
#define MAX3100_IRQ_PINSEL_VAL 1
#define MAX3100_IRQ_EINT 0
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

extern uint8_t max3100_status;
extern bool max3100_data_available;

/** I/O Buffers */
#define MAX3100_TX_BUF_LEN 256
#define MAX3100_RX_BUF_LEN 256

extern uint8_t max3100_tx_insert_idx, max3100_tx_extract_idx;
extern uint8_t max3100_rx_insert_idx, max3100_rx_extract_idx;

extern uint8_t max3100_tx_buf[MAX3100_TX_BUF_LEN];
extern uint8_t max3100_rx_buf[MAX3100_RX_BUF_LEN];

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

#define MAX3100_BAUD_RATE_19200 0x9 
#define MAX3100_BAUD_RATE_9600 0xA

#define MAX3100_BIT_NOT_RM (1U<<10)
#define MAX3100_BIT_NOT_TM (1U<<11)
#define MAX3100_BIT_NOT_FEN (1U<<13)
#define Max3100BitR(_conf_byte) ((_conf_byte)>>7)

/** Like Uart macros */
#define Uart3100Init() max3100_init()
#define Uart3100CheckFreeSpace(_x) (((int16_t)max3100_tx_extract_idx - max3100_tx_insert_idx + MAX3100_TX_BUF_LEN) % MAX3100_TX_BUF_LEN >= 1)
#define Uart3100Transmit(_x) max3100_putchar(_x)
#define Uart3100SendMessage() {}
#define Uart3100Getch() ({\
   uint8_t ret = max3100_rx_buf[max3100_rx_extract_idx]; \
   max3100_rx_extract_idx++; /* Since size=256 */        \
   ret;                                                 \
})

#define Uart3100ChAvailable() (max3100_rx_extract_idx != max3100_rx_insert_idx)

static inline void max3100_transmit(uint16_t data) {
  SSPDR = data >> 8;
  SSPDR = data & 0xff;
  SpiClrCPHA(); /* Data captured on first clock edge transition */
  Max3100Select();
  SpiEnable();
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
      && max3100_tx_extract_idx != max3100_tx_insert_idx) { 
    Max3100TransmitData(max3100_tx_buf[max3100_tx_extract_idx]);
    max3100_tx_extract_idx++; /* automatic overflow since len=256 */
    max3100_status == MAX3100_STATUS_WRITING;
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
extern void max3100_test_write( void );

static inline void max3100_event( void ) {
  if (max3100_status == MAX3100_STATUS_IDLE) {
    if (max3100_data_available)
      max3100_read_data();
    else
      max3100_flush();
  }
}


#endif /* MAX3100_H */
