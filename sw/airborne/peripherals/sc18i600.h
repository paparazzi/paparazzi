#ifndef SC18IS600_H
#define SC18IS600_H

#include "std.h"

/* Register definitions */
#define Sc18Is600_IOConfig 0x00
#define Sc18Is600_IOState  0x01
#define Sc18Is600_I2CClock 0x02
#define Sc18Is600_I2CTO    0x03
#define Sc18Is600_I2CStat  0x04
#define Sc18Is600_I2CAdr   0x05


enum Sc18Is600Status {
  Sc18Is600Idle,
  Sc18Is600SendingRequest,
  Sc18Is600WaitingForI2C,
  Sc18Is600ReadingI2CStat,
  Sc18Is600ReadingBuffer,
  Sc18Is600TransactionComplete,
};

enum Sc18Is600Transaction {
  Sc18Is600Transmit,
  Sc18Is600Receive,
  Sc18Is600Transcieve,
  Sc18Is600ReadRegister,
  Sc18Is600WriteRegister,
};
#define SC18IS600_BUF_LEN 96

struct Sc18Is600 {
  enum Sc18Is600Status status;
  enum Sc18Is600Transaction transaction;
  uint8_t priv_tx_buf[SC18IS600_BUF_LEN];
  uint8_t priv_rx_buf[SC18IS600_BUF_LEN];
  uint8_t rx_len;
  uint8_t i2c_status;
};

extern struct Sc18Is600 sc18is600;

extern void sc18is600_init(void);
extern void sc18is600_transmit(uint8_t addr, uint8_t len);
extern void sc18is600_receive(uint8_t addr, uint8_t len);
extern void sc18is600_tranceive(uint8_t addr, uint8_t len_tx, uint8_t len_rx);
extern void sc18is600_write_to_register(uint8_t addr, uint8_t value);
extern void sc18is600_read_from_register(uint8_t addr);

#include "peripherals/sc18is600_arch.h"
extern void sc18is600_arch_init(void);

#endif /* SC18IS600_H */
