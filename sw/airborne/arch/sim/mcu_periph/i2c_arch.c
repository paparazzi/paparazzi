#include "mcu_periph/i2c.h"

bool_t i2c_idle(struct i2c_periph *p __attribute__ ((unused))) { return TRUE; }
bool_t i2c_submit(struct i2c_periph* p __attribute__ ((unused)), struct i2c_transaction* t __attribute__ ((unused))) { return TRUE;}
void i2c_event(void) { }
void i2c2_setbitrate(int bitrate __attribute__ ((unused))) { }


#ifdef USE_I2C0
struct i2c_errors i2c0_errors;

void i2c0_hw_init(void) {
  i2c0.errors = &i2c0_errors;
  ZEROS_ERR_COUNTER(i2c0_errors);
}
#endif

#ifdef USE_I2C1
struct i2c_errors i2c1_errors;

void i2c1_hw_init(void) {
  i2c1.errors = &i2c1_errors;
  ZEROS_ERR_COUNTER(i2c1_errors);
}
#endif

#ifdef USE_I2C2
struct i2c_errors i2c2_errors;

void i2c2_hw_init(void) {
  i2c2.errors = &i2c2_errors;
  ZEROS_ERR_COUNTER(i2c2_errors);
}
#endif
