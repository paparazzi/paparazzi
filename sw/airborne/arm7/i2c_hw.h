#ifndef I2C_HW_H
#define I2C_HW_H


#include "LPC21xx.h"


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

#ifdef I2C0_STOP_HANDLER
#include I2C0_STOP_HANDLER_HEADER
#define I2c0StopHandler() I2C0_STOP_HANDLER()
#else 
#define I2c0StopHandler() {}
#endif /* I2C0_STOP_HANDLER */


extern void i2c0_hw_init(void);

#define I2c0SendAck()   { I2C0CONSET = _BV(AA); }
#define I2c0Finished()  {                                               \
    if (i2c0_finished) *i2c0_finished = TRUE;				\
    i2c0_status = I2C_IDLE;						\
    I2c0StopHandler();							\
}
#define I2c0SendStop()  {						\
    I2C0CONSET = _BV(STO);						\
    I2c0Finished();                                                     \
  }
#define I2c0SendStart() { I2C0CONSET = _BV(STA); }
#define I2c0SendByte(b) { I2C_DATA_REG = b; }

#define I2c0Receive(_ack) {	    \
    if (_ack) I2C0CONSET = _BV(AA); \
    else I2C0CONCLR = _BV(AAC);	    \
  }

#define I2c0ClearStart() { I2C0CONCLR = _BV(STAC); }
#define I2c0ClearIT() { I2C0CONCLR = _BV(SIC); }

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

extern void i2c1_hw_init(void);

#define I2c1StopHandler() {}

#define I2c1SendAck()   { I2C1CONSET = _BV(AA); }
#define I2c1SendStop()  {						\
    I2C1CONSET = _BV(STO);						\
    if (i2c1_finished) *i2c1_finished = TRUE;				\
    i2c1_status = I2C_IDLE;						\
    I2c1StopHandler();							\
  }
#define I2c1SendStart() { I2C1CONSET = _BV(STA); }
#define I2c1SendByte(b) { I2C1_DATA_REG = b; }

#define I2c1Receive(_ack) {	    \
    if (_ack) I2C1CONSET = _BV(AA); \
    else I2C1CONCLR = _BV(AAC);	    \
  }

#define I2c1ClearStart() { I2C1CONCLR = _BV(STAC); }
#define I2c1ClearIT() { I2C1CONCLR = _BV(SIC); }

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


extern void i2c0_hw_init(void);

#define I2c0SendAck()   { I2C0CONSET = _BV(AA); }
#define I2c0Finished()  {                                               \
    if (i2c0_finished) *i2c0_finished = TRUE;				\
    i2c0_status = I2C_IDLE;						\
}
#define I2c0SendStop()  {						\
    I2C0CONSET = _BV(STO);						\
    I2c0Finished();                                                     \
  }
#define I2c0SendStart() { I2C0CONSET = _BV(STA); }
#define I2c0SendByte(b) { I2C_DATA_REG = b; }

#define I2c0Receive(_ack) {	    \
    if (_ack) I2C0CONSET = _BV(AA); \
    else I2C0CONCLR = _BV(AAC);	    \
  }

#define I2c0ClearStart() { I2C0CONCLR = _BV(STAC); }
#define I2c0ClearIT() { I2C0CONCLR = _BV(SIC); }

#endif

#endif /* I2C_HW_H */
