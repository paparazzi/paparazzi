/*
 * $Id$
 *
 * Copyright (C) 2008-2009 Antoine Drouin <poinix@gmail.com>
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

#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "math/pprz_algebra_int.h"

#define IMU_ACCEL_X_NEUTRAL 32081
#define IMU_ACCEL_X_SENS -2.50411474
#define IMU_ACCEL_X_SENS_NUM -10650
#define IMU_ACCEL_X_SENS_DEN 4253

void test_1(void);
void test_2(void);
void test_3(void);

int main(int argc, char** argv){

  //  test_2();
  test_3();

  return 0;
}

void test_1(void) {

  double value_f;
  for (value_f=-9.81; value_f<9.81; value_f += 0.001) {

    double neutral_f = (double)IMU_ACCEL_X_NEUTRAL;
    double sensitivity_f = 1./IMU_ACCEL_X_SENS;

    double  sensor_raw_f = ACCEL_BFP_OF_REAL(value_f) * sensitivity_f + neutral_f;
    int32_t sensor_raw_i = rint(sensor_raw_f);

    double  scaled_sensor_f = ACCEL_BFP_OF_REAL(value_f);
#if 1
    int32_t scaled_sensor_i = ((sensor_raw_i - IMU_ACCEL_X_NEUTRAL) * IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN;
#endif

#if 0
    int32_t scaled_sensor_i = (sensor_raw_i * IMU_ACCEL_X_SENS_NUM / IMU_ACCEL_X_SENS_DEN) -
                              (IMU_ACCEL_X_NEUTRAL * IMU_ACCEL_X_SENS_NUM / IMU_ACCEL_X_SENS_DEN);
#endif

#if 0
    const int32_t delta = (sensor_raw_i - IMU_ACCEL_X_NEUTRAL);
    int32_t scaled_sensor_i;
    if (delta > 0)
      scaled_sensor_i = ( delta * IMU_ACCEL_X_SENS_NUM ) / IMU_ACCEL_X_SENS_DEN;
    else
      scaled_sensor_i = ( delta * IMU_ACCEL_X_SENS_NUM + (IMU_ACCEL_X_SENS_DEN>>1)) / IMU_ACCEL_X_SENS_DEN;
#endif

    printf("%f %f %d %f %d\n", value_f, sensor_raw_f, sensor_raw_i, scaled_sensor_f, scaled_sensor_i);



  }
}

void test_2(void) {

  int a;
  for (a=-7; a<7; a++) {
    int b = a/-2;
    int c = (a>0 ? a+1 : a-1)/-2;
    double d = rint((double)a/-2.);
    printf("%- d %- d %- d %- .1f\n", a, b, c, d);
  }

}

#define OFFSET_AND_ROUND(_a, _b) (((_a)+(1<<((_b)-1)))>>(_b))
#define OFFSET_AND_ROUND2(_a, _b) (((_a)+(1<<((_b)-1))-((_a)<0?1:0))>>(_b))
#define N_OFFSET 2
void test_3(void) {

 int a;
 for (a=-(1<<N_OFFSET); a<=(1<<N_OFFSET); a++) {
   int32_t b = (a>>N_OFFSET);
   int32_t c;
   //   if ( a>=0 )
   //     c = (a+(1<<(N_OFFSET-1)))>>N_OFFSET;
   //   else
   //     c = (a+(1<<(N_OFFSET-1))-1)>>N_OFFSET;
   c = OFFSET_AND_ROUND(a, N_OFFSET);

   int32_t d;
   d = OFFSET_AND_ROUND2(a, N_OFFSET);

   double e;
   e = (double)a/(double)(1<<N_OFFSET);

   printf("%- d %- d %- d %- d %.1f\n", a, b, c, d, e);
 }
}
