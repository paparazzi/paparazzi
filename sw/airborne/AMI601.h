#ifndef AMI601_H
#define AMI601_H

#include "std.h"
#include "i2c.h"

extern void ami601_init( void );

extern void ami601_read( void );
extern void ami601_periodic( void );
extern void ami601_scale_measures(void);

#define AMI601_NB_CHAN 6
extern uint16_t ami601_val[AMI601_NB_CHAN];
extern uint8_t ami601_foo1;
extern uint8_t ami601_foo2;
extern uint8_t ami601_foo3;

extern float ami601_ax;
extern float ami601_ay;
extern float ami601_az;

extern float ami601_mx;
extern float ami601_my;
extern float ami601_mz;

#define AMI601_IDLE            0
#define AMI601_SENDING_REQ     1
#define AMI601_WAITING_MEASURE 2
#define AMI601_READING_MEASURE 3
#define AMI601_DATA_AVAILABLE  4
extern volatile uint8_t ami601_status;
extern volatile bool_t ami601_i2c_done;
extern volatile uint32_t ami601_nb_err;

#define AMI601_SLAVE_ADDR 0x60


#define AMI601EventCheckAndHandle() {					\
    switch (ami601_status) {						\
    case AMI601_SENDING_REQ :						\
      if ( ami601_i2c_done ) {						\
	/* trigger delay for measurement */				\
	T0MR1 = T0TC + SYS_TICS_OF_USEC(4096);				\
	/* clear match 1 interrupt */					\
	T0IR = TIR_MR1I;						\
	/* enable match 1 interrupt */					\
	T0MCR |= TMCR_MR1_I;						\
	ami601_status =  AMI601_WAITING_MEASURE;			\
      }									\
      break;								\
    case AMI601_READING_MEASURE :					\
      if ( ami601_i2c_done ) {						\
	ami601_foo1 = i2c_buf[0]; /* AA ?  */				\
	ami601_foo2 = i2c_buf[1]; /* 55 ?  */				\
	ami601_foo3 = i2c_buf[2]; /* ERR ? */				\
	uint8_t i;							\
	for (i=0; i< AMI601_NB_CHAN; i++) {				\
	  ami601_val[i] = i2c_buf[3 + 2 * i];				\
	  ami601_val[i] += i2c_buf[3 + 2 * i + 1]  * 256;		\
	}								\
	ami601_status = AMI601_DATA_AVAILABLE;				\
      }									\
      break;								\
    }									\
  }

#define AMI601ReadMeasure() {						\
    /* disable match 1 interrupt */					\
    /*    T0MCR |= TMCR_MR1_I;   */					\
    ami601_i2c_done = FALSE;						\
    ami601_status =  AMI601_READING_MEASURE;				\
    i2c_receive(AMI601_SLAVE_ADDR, 15, &ami601_i2c_done);		\
  }

#endif /* AMI601_H */
