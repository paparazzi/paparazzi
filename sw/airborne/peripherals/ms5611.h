/*
 *
 * Copyright (C) 2012 Piotr Esden-Tempski
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

/* Register definition for MS5611
 */

#ifndef MS5611_H
#define MS5611_H

/* default i2c address
 * when CSB is set to GND ADDR0 is valid
 * when CSB is set to VCC ADDR1 is valid
 *
 * Note: Aspirin 2.1 has CSB bound to GND.
 */
#define MS5611_ADDR0 0x77
#define MS5611_ADDR1 0x76

/* General Registers */
#define MS5611_REG_ADCREAD   0x00 // Read converted value
#define MS5611_REG_RESET     0x1E // Reset command

/* OSR definitions */
#define MS5611_OSR256        0x02
#define MS5611_OSR512        0x02
#define MS5611_OSR1024       0x04
#define MS5611_OSR2048       0x06
#define MS5611_OSR4096       0x08

/* D1 Register defines */
#define MS5611_REG_D1R       0x40 // Request D1 (pressure) conversion
#define MS5611_REG_D1(_osr)  (MS5611_REG_D1R | _osr)
#define MS5611_REG_D1OSR256  MS5611_REG_D1(MS5611_ORS256)
#define MS5611_REG_D1OSR512  MS5611_REG_D1(MS5611_OSR512)
#define MS5611_REG_D1OSR1024 MS5611_REG_D1(MS5611_OSR1024)
#define MS5611_REG_D1OSR2048 MS5611_REG_D1(MS5611_OSR2048)
#define MS5611_REG_D1OSR4096 MS5611_REG_D1(MS5611_OSR4096)

/* D2 register defines */
#define MS5611_REG_D2R       0x50 // Request D2 (temperature) conversion
#define MS5611_REG_D2(_osr)  (MS5611_REG_D2R | _osr)
#define MS5611_REG_D2OSR256  MS5611_REG_D2(MS5611_ORS256)
#define MS5611_REG_D2OSR512  MS5611_REG_D2(MS5611_OSR512)
#define MS5611_REG_D2OSR1024 MS5611_REG_D2(MS5611_OSR1024)
#define MS5611_REG_D2OSR2048 MS5611_REG_D2(MS5611_OSR2048)
#define MS5611_REG_D2OSR4096 MS5611_REG_D2(MS5611_OSR4096)

/* PROM address definitions */
#define MS5611_PROM_ADDR0    0x00 // 16 bit reserved for manufacturer
#define MS5611_PROM_ADDR1    0x02 // Coefficient 1 (Pressure Sensitivity | SENS_T1)
#define MS5611_PROM_ADDR2    0x04 // Coefficient 2 (Pressure Offset | OFF_T1)
#define MS5611_PROM_ADDR3    0x06 // Coefficient 3 (Temperature coefficient of pressure sensitivity | TCS)
#define MS5611_PROM_ADDR4    0x08 // Coefficient 4 (Temperature coefficient of pressure offset | TCO)
#define MS5611_PROM_ADDR5    0x0A // Coefficient 5 (Reference temperature | T_REF)
#define MS5611_PROM_ADDR6    0x0C // Coefficient 6 (Temperature coefficient of the temperature | TEMPSENS)
#define MS5611_PROM_ADDR7    0x0E // CRC-4 (Chkecksum of the PROM content)

/* PROM register defines */
#define MS5611_REG_PROMR     0xA0
#define MS5611_REG_PROM(_addr) (MS5611_REG_PROMR | _addr)
#define MS5611_REG_C1        MS5611_REG_PROM(ADDR1)
#define MS5611_REG_C2        MS5611_REG_PROM(ADDR2)
#define MS5611_REG_C3        MS5611_REG_PROM(ADDR3)
#define MS5611_REG_C4        MS5611_REG_PROM(ADDR4)
#define MS5611_REG_C5        MS5611_REG_PROM(ADDR5)
#define MS5611_REG_C6        MS5611_REG_PROM(ADDR6)
#define MS5611_REG_CRC       MS5611_REG_PROM(ADDR7)

#endif /* MS5611_H */
