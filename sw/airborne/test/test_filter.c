/*
 * Copyright (C) 2024 Ewoud Smeur <e.j.j.smeur@tudelft.nl>
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
 * @file test_filter.c
 *
 * Test stability of Butterworth 2nd order low pass filter
 */

#include <stdio.h>
#include "std.h"
#include "filters/low_pass_filter.h"


Butterworth2LowPass testfilter;

void print_filter_coef(Butterworth2LowPass *filter);


void test_butterworth(float cutoff_hz, float periodic_frequency, int16_t num_samples);

int main(int argc, char **argv)
{
  float cutoff_hz = 0.1f; //hz
  float periodic_frequency = 500; //hz
  int16_t num_samples = 15000;
  test_butterworth(cutoff_hz, periodic_frequency, num_samples);
  print_filter_coef(&testfilter);
}

/*
 * function to test filter stability
 */
void test_butterworth(float cutoff_hz, float periodic_frequency, int16_t num_samples)
{
  float init_value = 500.f;

  // tau = 1/(2*pi*Fc)
  float tau = 1.0 / (2.0 * M_PI * cutoff_hz);
  float sample_time = 1.0 / periodic_frequency;
  // Filter init
  init_butterworth_2_low_pass(&testfilter, tau, sample_time, init_value);

  print_filter_coef(&testfilter);

  float sine_freq = 1.f/10.f;
  float value = 0.f;
  for (int i = 0; i < num_samples; i++)
  {
    value = sinf(i * 2 * M_PI * sine_freq / periodic_frequency);

    float filtered_value = update_butterworth_2_low_pass(&testfilter, value);
    printf("value: %f, filtered value: %f\n", value, filtered_value);
  }
}

void print_filter_coef(Butterworth2LowPass *filter)
{
  printf("filter coefficients and state:\n");
  printf("a = %10e, %10e\n", filter->a[0], filter->a[1]);
  printf("b = %10e, %10e\n", filter->b[0], filter->b[1]);
  printf("i = %f, %f\n", filter->i[0], filter->i[1]);
  printf("o = %f, %f\n", filter->o[0], filter->o[1]);
}

void print_filter_values(Butterworth2LowPass *filter)
{
  printf("filter values:\n");
  printf("i = %f, %f\n", filter->i[0], filter->i[1]);
  printf("o = %f, %f\n", filter->o[0], filter->o[1]);
}