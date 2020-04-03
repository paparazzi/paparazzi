/*
 * Copyright (C) 2012 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file arch/sim/mcu_periph/i2c_arch.c
 * Dummy functions for handling of I2C hardware in sim.
 */

#include "mcu_periph/i2c.h"


static bool __attribute__((unused)) i2c_sim_idle(struct i2c_periph *p __attribute__((unused))) { return true; }
static bool __attribute__((unused)) i2c_sim_submit(struct i2c_periph *p __attribute__((unused)), struct i2c_transaction *t __attribute__((unused))) { return true;}
static void __attribute__((unused)) i2c_sim_setbitrate(struct i2c_periph *p __attribute__((unused)), int bitrate __attribute__((unused))) { }
void i2c_event(void) { }

#if USE_I2C0
struct i2c_errors i2c0_errors;

void i2c0_hw_init(void)
{
  i2c0.idle = i2c_sim_idle;
  i2c0.submit = i2c_sim_submit;
  i2c0.setbitrate = i2c_sim_setbitrate;
  i2c0.errors = &i2c0_errors;
  ZEROS_ERR_COUNTER(i2c0_errors);
}

#endif

#if USE_I2C1
struct i2c_errors i2c1_errors;

void i2c1_hw_init(void)
{
  i2c1.idle = i2c_sim_idle;
  i2c1.submit = i2c_sim_submit;
  i2c1.setbitrate = i2c_sim_setbitrate;
  i2c1.errors = &i2c1_errors;
  ZEROS_ERR_COUNTER(i2c1_errors);
}

#endif

#if USE_I2C2
struct i2c_errors i2c2_errors;

void i2c2_hw_init(void)
{
  i2c2.idle = i2c_sim_idle;
  i2c2.submit = i2c_sim_submit;
  i2c2.setbitrate = i2c_sim_setbitrate;
  i2c2.errors = &i2c2_errors;
  ZEROS_ERR_COUNTER(i2c2_errors);
}

#endif
