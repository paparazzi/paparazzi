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


#ifdef USE_I2C0

extern void i2c0_hw_init(void);

#endif /* USE_I2C0 */



#ifdef USE_I2C1

extern void i2c1_hw_init(void);

#endif /* USE_I2C1 */


#endif /* I2C_HW_H */
