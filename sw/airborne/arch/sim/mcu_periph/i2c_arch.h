#ifndef SIM_MCU_PERIPH_I2C_ARCH_H
#define SIM_MCU_PERIPH_I2C_ARCH_H

#define I2cSendStart() {}

//extern void i2c_hw_init(void);
#define i2c0_hw_init() {}
#define i2c1_hw_init() {}
#define i2c2_hw_init() {}

#define I2c0SendStart() {}
#define I2c1SendStart() {}
#define I2c2SendStart() {}

#define i2c1_ev_irq_handler() {}
#define i2c1_er_irq_handler() {};

#define i2c2_ev_irq_handler() {}
#define i2c2_er_irq_handler() {};

#endif /* SIM_MCU_PERIPH_I2C_ARCH_H */
