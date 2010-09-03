#ifndef I2C_H
#define I2C_H

#include "std.h"

#include "i2c_hw.h"

enum I2CTransactionType { 
  I2CTransTx, 
  I2CTransRx, 
  I2CTransTxRx 
};

enum I2CTransactionStatus {  
  I2CTransPending, 
  I2CTransRunning, 
  I2CTransSuccess, 
  I2CTransFailed
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
  bool_t   stop_after_transmit;
  volatile uint8_t  buf[I2C_BUF_LEN];
  volatile enum I2CTransactionStatus status;
};

#define I2C_TRANSACTION_QUEUE_LEN 8

struct i2c_periph {
  /* circular buffer holding transactions */
  struct i2c_transaction* trans[I2C_TRANSACTION_QUEUE_LEN];
  uint8_t trans_insert_idx;
  uint8_t trans_extract_idx;
  /* internal state of the peripheral */
  volatile enum I2CStatus status;
  volatile uint8_t idx_buf;
  void* reg_addr;
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
#define I2C_ZERO_EVENTS(_err) {					    \
    _err.irq_cnt = 0;						    \
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

extern void i2c0_init(void);
extern void i2c0_receive(uint8_t slave_addr, uint16_t len, volatile bool_t* finished);
extern void i2c0_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished);
extern void i2c0_transmit_no_stop(uint8_t slave_addr, uint8_t len, volatile bool_t* finished);
extern void i2c0_transceive(uint8_t slave_addr, uint8_t len_w, uint16_t len_r, volatile bool_t* finished);

extern volatile uint8_t i2c0_status;
extern volatile bool_t  i2c0_stop_after_transmit;


#ifndef I2C0_BUF_LEN
#define I2C0_BUF_LEN 32
#endif

extern volatile uint8_t i2c0_buf[I2C0_BUF_LEN];
extern volatile uint16_t i2c0_len_r;
extern volatile uint8_t  i2c0_len_w;
extern volatile uint8_t i2c0_index;
extern volatile uint8_t i2c0_slave_addr;
extern volatile uint8_t i2c0_trx;

extern volatile bool_t* i2c0_finished;

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
extern bool_t i2c_submit(struct i2c_periph* p, struct i2c_transaction* t);

#endif /* I2C_H */
