#ifndef MCU_PERIPH_I2C_H
#define MCU_PERIPH_I2C_H

#include "std.h"

#include "mcu_periph/i2c_arch.h"

enum I2CTransactionType {
  I2CTransTx,
  I2CTransRx,
  I2CTransTxRx
};

enum I2CTransactionStatus {
  I2CTransPending,
  I2CTransRunning,
  I2CTransSuccess,
  I2CTransFailed,
  I2CTransDone
};

enum I2CStatus {
  I2CIdle,
  I2CStartRequested,
  I2CAddrWrSent,
  I2CAddrRdSent,
  I2CSendingByte,
  /*  I2CSendingLastByte, */
  I2CReadingByte,
  I2CReadingLastByte,
  I2CStopRequested,
  I2CRestartRequested,
  I2CComplete,
  I2CFailed
};

#ifndef I2C_BUF_LEN
#define I2C_BUF_LEN 32
#endif

struct i2c_transaction {
  enum I2CTransactionType type;
  uint8_t  slave_addr;
  uint16_t len_r;
  uint8_t  len_w;
  volatile uint8_t  buf[I2C_BUF_LEN];
  volatile enum I2CTransactionStatus status;
};

#ifndef I2C_TRANSACTION_QUEUE_LEN
#define I2C_TRANSACTION_QUEUE_LEN 8
#endif

struct i2c_periph {
  /* circular buffer holding transactions */
  struct i2c_transaction* trans[I2C_TRANSACTION_QUEUE_LEN];
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /* internal state of the peripheral */
  volatile enum I2CStatus status;
  volatile uint8_t idx_buf;
  void* reg_addr;
  void *init_struct;
  uint16_t scl_pin;
  uint16_t sda_pin;
  struct i2c_errors *errors;
};


struct i2c_errors {
  volatile uint16_t ack_fail_cnt;
  volatile uint16_t miss_start_stop_cnt;
  volatile uint16_t arb_lost_cnt;
  volatile uint16_t over_under_cnt;
  volatile uint16_t pec_recep_cnt;
  volatile uint16_t timeout_tlow_cnt;
  volatile uint16_t smbus_alert_cnt;
  volatile uint16_t unexpected_event_cnt;
  volatile uint32_t last_unexpected_event;
  volatile uint32_t er_irq_cnt;
  volatile uint32_t irq_cnt;
  volatile uint32_t event_chain[16];
  volatile enum I2CStatus status_chain[16];
};


#include <string.h>
#define I2C_ZERO_EVENTS(_err) {                     \
    _err.irq_cnt = 0;                           \
    memset((void*)_err.event_chain, 0, sizeof(_err.event_chain));   \
    memset((void*)_err.status_chain, 0, sizeof(_err.status_chain)); \
  }

#define ZEROS_ERR_COUNTER(_i2c_err) {			\
    _i2c_err.ack_fail_cnt = 0;				\
    _i2c_err.miss_start_stop_cnt = 0;			\
    _i2c_err.arb_lost_cnt = 0;				\
    _i2c_err.over_under_cnt = 0;			\
    _i2c_err.pec_recep_cnt = 0;				\
    _i2c_err.timeout_tlow_cnt = 0;			\
    _i2c_err.smbus_alert_cnt = 0;			\
    _i2c_err.unexpected_event_cnt = 0;			\
    _i2c_err.last_unexpected_event = 0;			\
    _i2c_err.er_irq_cnt = 0;				\
  }


#ifdef USE_I2C0

extern struct i2c_periph i2c0;
extern void i2c0_init(void);

#endif /* USE_I2C0 */



#ifdef USE_I2C1

extern struct i2c_periph i2c1;
extern void i2c1_init(void);

#endif /* USE_I2C1 */



#ifdef USE_I2C2

extern struct i2c_periph i2c2;
extern void i2c2_init(void);

#endif /* USE_I2C2 */

extern void   i2c_init(struct i2c_periph* p);
extern bool_t i2c_idle(struct i2c_periph* p);
extern bool_t i2c_submit(struct i2c_periph* p, struct i2c_transaction* t);

#define I2CReceive(_p, _t, _s_addr, _len) { \
  _t.type = I2CTransRx;                     \
  _t.slave_addr = _s_addr;                  \
  _t.len_r = _len;                          \
  _t.len_w = 0;                             \
  i2c_submit(&(_p),&(_t));                  \
}

#define I2CTransmit(_p, _t, _s_addr, _len) {	\
  _t.type = I2CTransTx;			                  \
  _t.slave_addr = _s_addr;			              \
  _t.len_r = 0;				                        \
  _t.len_w = _len;				                    \
  i2c_submit(&(_p),&(_t));			              \
}

#define I2CTransceive(_p, _t, _s_addr, _len_w, _len_r) {  \
  _t.type = I2CTransTxRx;                                 \
  _t.slave_addr = _s_addr;                                \
  _t.len_r = _len_r;                                      \
  _t.len_w = _len_w;                                      \
  i2c_submit(&(_p),&(_t));                                \
}


#endif /* I2C_H */
