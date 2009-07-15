#ifndef I2C_HW_H
#define I2C_HW_H


#include "LPC21xx.h"

#ifdef USE_I2C0

#ifdef USE_BUSS_TWI_BLMC
#include "actuators_buss_twi_blmc_hw.h"
#define I2c0StopHandler() ActuatorsBussTwiBlmcNext()
#else

#ifdef USE_BUSS_TWI_BLMC_MOTOR
#include "buss_twi_blmc_hw.h"
#define I2c0StopHandler() BussTwiBlmcNext()
#else
#define I2c0StopHandler() {}
#endif

#endif



extern void i2c0_hw_init(void);

#define I2c0SendAck()   { I2C0CONSET = _BV(AA); }
#define I2c0SendStop()  {						\
    I2C0CONSET = _BV(STO);						\
    if (i2c0_finished) *i2c0_finished = TRUE;				\
    i2c0_status = I2C_IDLE;						\
    I2c0StopHandler();							\
  }
#define I2c0SendStart() { I2C0CONSET = _BV(STA); }
#define I2c0SendByte(b) { I2C_DATA_REG = b; }

#define I2c0Receive(_ack) {	    \
    if (_ack) I2C0CONSET = _BV(AA); \
    else I2C0CONCLR = _BV(AAC);	    \
  }

#define I2c0ClearStart() { I2C0CONCLR = _BV(STAC); }
#define I2c0ClearIT() { I2C0CONCLR = _BV(SIC); }

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

#endif /* USE_I2C1 */

#endif /* I2C_HW_H */
