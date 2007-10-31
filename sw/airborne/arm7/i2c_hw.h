#ifndef I2C_HW_H
#define I2C_HW_H


#include "LPC21xx.h"

#ifdef USE_BUSS_TWI_BLMC
#include "actuators_buss_twi_blmc_hw.h"
#define I2cStopHandler() ActuatorsBussTwiBlmcNext()
#else
#define I2cStopHandler() {}
#endif

extern void i2c_hw_init(void);

#define I2cSendAck()   { I2C0CONSET = _BV(AA); }
#define I2cSendStop()  {						\
    I2C0CONSET = _BV(STO);						\
    if (i2c_finished) *i2c_finished = TRUE;				\
    i2c_status = I2C_IDLE;						\
    I2cStopHandler();							\
  }
#define I2cSendStart() { I2C0CONSET = _BV(STA); }
#define I2cSendByte(b) { I2C_DATA_REG = b; }

#define I2cReceive(_ack) {	    \
    if (_ack) I2C0CONSET = _BV(AA); \
    else I2C0CONCLR = _BV(AAC);	    \
  }

#define I2cClearStart() { I2C0CONCLR = _BV(STAC); }
#define I2cClearIT() { I2C0CONCLR = _BV(SIC); }


#endif /* I2C_HW_H */
