#include "mcu_periph/i2c.h"

void i2c_hw_init ( void ) {}

bool_t i2c_idle(struct i2c_periph *p __attribute__ ((unused))) { return TRUE; }
bool_t i2c_submit(struct i2c_periph* p __attribute__ ((unused)), struct i2c_transaction* t __attribute__ ((unused))) { return TRUE;}
