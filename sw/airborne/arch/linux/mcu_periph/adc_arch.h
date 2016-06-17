/*
 * Copyright (C) 2015 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * Paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/**
 * @file arch/linux/mcu_periph/adc_arch.h
 * @ingroup linux_arch
 *
 * Driver for the analog to digital converters in Linux based systems..
 */

#ifndef ADC_ARCH_H
#define ADC_ARCH_H

/* Main ADC structure */
struct adc_t {
  uint8_t dev_id;         ///< The iio device ID
  uint8_t *channels;      ///< Channels used in the iio device
  uint8_t channels_cnt;   ///< Amount of channels
  uint16_t buf_length;    ///< ADC buffer length
};

#if USE_ADC0
extern struct adc_t adc0;
#endif

#if USE_ADC1
extern struct adc_t adc1;
#endif

void adc_enable(struct adc_t *adc, uint8_t value);
int adc_read(struct adc_t *adc, uint16_t *buf, uint16_t size);

#endif /* ADC_ARCH_H */
