#include <stdio.h>
#include <math.h>
#include <inttypes.h>

#include "booz_geometry_mixed.h"

#define IMU_ACCEL_X_NEUTRAL 32081
#define IMU_ACCEL_X_SENS -2.50411474
#define IMU_ACCEL_X_SENS_NUM -10650
#define IMU_ACCEL_X_SENS_DEN 4253

void test_1(void);
void test_2(void);

int main(int argc, char** argv){

  test_2();

  return 0;
}

void test_1(void) {

  double value_f;
  for (value_f=-9.81; value_f<9.81; value_f += 0.001) {

    double neutral_f = (double)IMU_ACCEL_X_NEUTRAL;
    double sensitivity_f = 1./IMU_ACCEL_X_SENS;
    
    double  sensor_raw_f = BOOZ_ACCEL_I_OF_F(value_f) * sensitivity_f + neutral_f;
    int32_t sensor_raw_i = rint(sensor_raw_f);
    
    double  scaled_sensor_f = BOOZ_ACCEL_I_OF_F(value_f);
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
