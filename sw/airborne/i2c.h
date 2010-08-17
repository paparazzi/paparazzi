#ifndef I2C_H
#define I2C_H

#include "std.h"

#include "i2c_hw.h"

enum I2CTransaction { 
  I2CTransTx, 
  I2CTransRx, 
  I2CTransTxRx 
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

struct i2c {
  volatile enum I2CStatus status;
  volatile enum I2CTransaction transaction;
  volatile uint8_t  slave_addr;
  volatile uint16_t len_r;
  volatile uint8_t  len_w;
  volatile bool_t   stop_after_transmit;
  volatile uint8_t  index;
  volatile bool_t*  finished;
  volatile uint8_t  buf[I2C_BUF_LEN];
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


#define I2C_START        0x08
#define I2C_RESTART      0x10
#define I2C_MT_SLA_ACK   0x18
#define I2C_MT_SLA_NACK  0x20
#define I2C_MT_DATA_ACK  0x28
#define I2C_MR_SLA_ACK   0x40
#define I2C_MR_SLA_NACK  0x48
#define I2C_MR_DATA_ACK  0x50
#define I2C_MR_DATA_NACK 0x58


#define I2C_IDLE 0
#define I2C_BUSY 1

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

#define I2c0Automaton(state) {						\
    switch (state) {							\
    case I2C_START:							\
    case I2C_RESTART:							\
      I2c0SendByte(i2c0_slave_addr);					\
      I2c0ClearStart();							\
      i2c0_index = 0;							\
      break;								\
    case I2C_MR_DATA_ACK:						\
      if (i2c0_index < i2c0_len_r) {					\
	i2c0_buf[i2c0_index] = I2C_DATA_REG;				\
	i2c0_index++;							\
	I2c0Receive(i2c0_index < i2c0_len_r - 1);			\
      }									\
      else {								\
	/* error , we should have got NACK */				\
	I2c0SendStop();							\
      }									\
      break;								\
    case I2C_MR_SLA_ACK: /* At least one char */			\
      /* Wait and reply with ACK or NACK */				\
      I2c0Receive(i2c0_index < i2c0_len_r - 1);				\
      break;								\
    case I2C_MR_SLA_NACK:						\
    case I2C_MT_SLA_NACK:						\
      I2c0SendStart();							\
      break;								\
    case I2C_MT_SLA_ACK:						\
    case I2C_MT_DATA_ACK:						\
      if (i2c0_index < i2c0_len_w) {					\
	I2c0SendByte(i2c0_buf[i2c0_index]);				\
	i2c0_index++;							\
      } else {								\
        if (i2c0_trx) {							\
          i2c0_trx = 0;							\
          i2c0_index = 0;						\
          i2c0_slave_addr |= 1;						\
          I2c0SendStart();						\
        } else {							\
	  if (i2c0_stop_after_transmit) {				\
            I2c0SendStop();						\
          } else {							\
            I2c0Finished();						\
          }								\
	}								\
      }									\
      break;								\
    case I2C_MR_DATA_NACK:						\
      if (i2c0_index < i2c0_len_r) {					\
	i2c0_buf[i2c0_index] = I2C_DATA_REG;				\
      }									\
      I2c0SendStop();							\
      break;								\
    default:								\
      I2c0SendStop();							\
      /* FIXME log error */						\
    }									\
  }									\

#endif /* USE_I2C0 */   

#ifdef USE_I2C1

extern void i2c1_init(void);
extern void i2c1_receive(uint8_t slave_addr, uint16_t len, volatile bool_t* finished);
extern void i2c1_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished);
extern void i2c1_transceive(uint8_t slave_addr, uint8_t len_w, uint16_t len_r, volatile bool_t* finished);

extern volatile uint8_t i2c1_status;
extern struct i2c i2c1;

#ifndef I2C1_BUF_LEN
#define I2C1_BUF_LEN 16
#endif

extern volatile uint8_t i2c1_buf[I2C1_BUF_LEN];
extern volatile uint16_t i2c1_len_r;
extern volatile uint8_t  i2c1_len_w;
extern volatile uint16_t i2c1_index;
extern volatile uint8_t i2c1_slave_addr;
extern volatile uint8_t i2c1_trx;

extern volatile bool_t* i2c1_finished;

#define I2c1Automaton(state) {						\
    switch (state) {							\
    case I2C_START:							\
    case I2C_RESTART:							\
      I2c1SendByte(i2c1_slave_addr);					\
      I2c1ClearStart();							\
      i2c1_index = 0;							\
      break;								\
    case I2C_MR_DATA_ACK:						\
      if (i2c1_index < i2c1_len_r) {					\
	i2c1_buf[i2c1_index] = I2C1_DATA_REG;				\
	i2c1_index++;							\
	I2c1Receive(i2c1_index < i2c1_len_r - 1);			\
      }									\
      else {								\
	/* error , we should have got NACK */				\
	I2c1SendStop();							\
      }									\
      break;								\
    case I2C_MR_SLA_ACK: /* At least one char */			\
      /* Wait and reply with ACK or NACK */				\
      I2c1Receive(i2c1_index < i2c1_len_r - 1);				\
      break;								\
    case I2C_MR_SLA_NACK:						\
    case I2C_MT_SLA_NACK:						\
      I2c1SendStart();							\
      break;								\
    case I2C_MT_SLA_ACK:						\
    case I2C_MT_DATA_ACK:						\
      if (i2c1_index < i2c1_len_w) {					\
	I2c1SendByte(i2c1_buf[i2c1_index]);				\
	i2c1_index++;							\
      } else {								\
        if (i2c1_trx) {							\
          i2c1_trx = 0;							\
          i2c1_index = 0;						\
          i2c1_slave_addr |= 1;						\
          I2c1SendStart();						\
        } else {							\
	  I2c1SendStop();						\
	}								\
      }									\
      break;								\
    case I2C_MR_DATA_NACK:						\
      if (i2c1_index < i2c1_len_r) {					\
	i2c1_buf[i2c1_index] = I2C1_DATA_REG;				\
      }									\
      I2c1SendStop();							\
      break;								\
    default:								\
      I2c1SendStop();							\
      /* LED_ON(2); FIXME log error */					\
    }									\
  }									\
   
#endif /* USE_I2C1 */



#ifdef USE_I2C2


extern struct i2c i2c2;

extern void i2c2_init(void);
extern void i2c2_receive(uint8_t slave_addr, uint8_t len, volatile bool_t* finished);
extern void i2c2_transmit(uint8_t slave_addr, uint8_t len, volatile bool_t* finished);
extern void i2c2_transceive(uint8_t slave_addr, uint8_t len_w, uint16_t len_r, volatile bool_t* finished);

#endif /* USE_I2C2 */


#endif /* I2C_H */
