#ifndef SIM_MCU_PERIPH_I2C_ARCH_H
#define SIM_MCU_PERIPH_I2C_ARCH_H

#include "mcu_periph/i2c.h"

#define I2cSendStart() {}


#ifdef USE_I2C0

extern void i2c0_hw_init(void);
#define i2c0_ev_irq_handler() {}
#define i2c0_er_irq_handler() {}

#endif /* USE_I2C0 */


#ifdef USE_I2C1

extern void i2c1_hw_init(void);
#define i2c1_ev_irq_handler() {}
#define i2c1_er_irq_handler() {}

#endif /* USE_I2C1 */


#ifdef USE_I2C2

extern void i2c2_hw_init(void);
#define i2c2_ev_irq_handler() {}
#define i2c2_er_irq_handler() {}

#endif /* USE_I2C2 */


#endif /* SIM_MCU_PERIPH_I2C_ARCH_H */
