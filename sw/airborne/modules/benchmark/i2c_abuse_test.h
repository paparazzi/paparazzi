/*
 * Copyright (C) 2011  Christophe De Wagter
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
 *
 */

/** @file modules/benchmark/i2c_abuse_test.h
 *
 * Total I2C Abuse:
 *
 * -all transaction types: T1 T2 T3 T4 R1 R2 R3 R4 T1R1 T2R1 T1R2 T1R3 T1R4 T1R5 T2R5
 * -all bitrates: 1k (way too slow) to 1M (way to fast)
 * -occasional Short circuit (simulate bus capacitance or EMI errors)
 * -variable bus load: from empty to full stack
 *
 * -Connect LED to MosFet that pulls-down the SCL and SDA lines
 */

#ifndef I2C_ABUSE_TEST_MODULE_H
#define I2C_ABUSE_TEST_MODULE_H

#ifndef I2C_ABUSE_SHORT_SCL_LED
#define I2C_ABUSE_SHORT_SCL_LED 2
#endif

#ifndef I2C_ABUSE_SHORT_SDA_LED
#define I2C_ABUSE_SHORT_SDA_LED 3
#endif

void init_i2c_abuse_test(void);
void event_i2c_abuse_test(void);
void periodic_50Hz_i2c_abuse_test(void);

#endif
