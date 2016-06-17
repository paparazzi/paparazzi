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


bool i2c_idle(struct i2c_periph *p __attribute__((unused))) { return true; }
bool i2c_submit(struct i2c_periph *p __attribute__((unused)), struct i2c_transaction *t __attribute__((unused))) { return true;}
void i2c_event(void) { }
void i2c2_setbitrate(int bitrate __attribute__((unused))) { }

#if USE_I2C0
struct i2c_errors i2c0_errors;

void i2c0_hw_init(void)
{
  i2c0.errors = &i2c0_errors;
  ZEROS_ERR_COUNTER(i2c0_errors);
}

void i2c0_ev_isr(void)
{
}

void i2c0_er_isr(void)
{
}
#endif

#if USE_I2C1
struct i2c_errors i2c1_errors;

void i2c1_hw_init(void)
{
  i2c1.errors = &i2c1_errors;
  ZEROS_ERR_COUNTER(i2c1_errors);
}

void i2c1_ev_isr(void)
{
}

void i2c1_er_isr(void)
{
}
#endif

#if USE_I2C2
struct i2c_errors i2c2_errors;

void i2c2_hw_init(void)
{
  i2c2.errors = &i2c2_errors;
  ZEROS_ERR_COUNTER(i2c2_errors);
}

void i2c2_ev_isr(void)
{
}

void i2c2_er_isr(void)
{
}
#endif
