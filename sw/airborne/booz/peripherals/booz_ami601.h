#ifndef BOOZ_AMI601_H
#define BOOZ_AMI601_H

#include "std.h"
#include "i2c.h"

extern void ami601_init( void );

extern void ami601_read( void );
extern void ami601_periodic( void );
extern void ami601_scale_measures(void);

#define AMI601_NB_CHAN 6
extern uint16_t ami601_values[AMI601_NB_CHAN];
extern uint8_t ami601_foo1;
extern uint8_t ami601_foo2;
extern uint8_t ami601_foo3;

#define AMI601_IDLE            0
#define AMI601_SENDING_REQ     1
#define AMI601_WAITING_MEASURE 2
#define AMI601_READING_MEASURE 3
#define AMI601_DATA_AVAILABLE  4
extern volatile uint8_t  ami601_status;
extern struct i2c_transaction  ami601_i2c_trans;
extern volatile uint32_t ami601_nb_err;

#define AMI601_SLAVE_ADDR 0x60

#ifdef SITL
#define AMI601Event(_handler) {			\
    if (ami601_status == AMI601_DATA_AVAILABLE)	\
      _handler();				\
  }
#else
#define AMI601Event(_handler) {						\
    switch (ami601_status) {						\
    case AMI601_SENDING_REQ :						\
      if ( ami601_i2c_trans.status == I2CTransSuccess ) {		\
	/* trigger delay for measurement */				\
	T0MR1 = T0TC + SYS_TICS_OF_USEC(12288);				\
	/* clear match 1 interrupt */					\
	T0IR = TIR_MR1I;						\
	/* enable match 1 interrupt */					\
	T0MCR |= TMCR_MR1_I;						\
	ami601_status =  AMI601_WAITING_MEASURE;			\
      }									\
      break;								\
    case AMI601_READING_MEASURE :					\
      if ( ami601_i2c_trans.status == I2CTransSuccess ) {		\
	ami601_foo1 = ami601_i2c_trans.buf[0]; /* AA ?  */		\
	ami601_foo2 = ami601_i2c_trans.buf[1]; /* 55 ?  */		\
	ami601_foo3 = ami601_i2c_trans.buf[2]; /* ERR ? */		\
	uint8_t i;							\
	for (i=0; i< AMI601_NB_CHAN; i++) {				\
	  ami601_values[i] = ami601_i2c_trans.buf[3 + 2 * i];		\
	  ami601_values[i] += ami601_i2c_trans.buf[3 + 2 * i + 1] * 256; \
	}								\
	ami601_status = AMI601_DATA_AVAILABLE;				\
	_handler();							\
      }									\
      break;								\
    }									\
  }
#endif

#define AMI601_IT TIR_MR1I
#define AMI601_ISR()  AMI601ReadMeasure()
#define AMI601ReadMeasure() {						\
    ami601_status =  AMI601_READING_MEASURE;				\
    ami601_i2c_trans.type = I2CTransRx;					\
    ami601_i2c_trans.len_r = 15;					\
    ami601_i2c_trans.slave_addr = AMI601_SLAVE_ADDR;			\
    i2c_submit(&i2c1, &ami601_i2c_trans);				\
  }

#endif /* AMI601_H */
