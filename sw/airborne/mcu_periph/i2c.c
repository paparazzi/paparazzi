#include "mcu_periph/i2c.h"

#ifdef USE_I2C0

struct i2c_periph i2c0;

void i2c0_init(void) {
  i2c_init(&i2c0);
  i2c0_hw_init();
}

#endif /* USE_I2C0 */


#ifdef USE_I2C1

struct i2c_periph i2c1;

void i2c1_init(void) {
  i2c_init(&i2c1);
  i2c1_hw_init();
}

#endif /* USE_I2C1 */


#ifdef USE_I2C2

struct i2c_periph i2c2;

void i2c2_init(void) {
  i2c_init(&i2c2);
  i2c2_hw_init();
}

#endif /* USE_I2C2 */

void   i2c_init(struct i2c_periph* p) {
  p->trans_insert_idx = 0;
  p->trans_extract_idx = 0;
  p->status = I2CIdle;
}


