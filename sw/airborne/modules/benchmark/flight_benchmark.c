//Author: Bruzzlee

//This module allows a quantitative assessment of the flight.
//It calculates the sum of squared error of the two-dimensional course (x / y), the altitude and true airspeed.
//The sum of squared error of the course and altitude were separated, because they are regulated separately, and so they dependent on various parameters.
//The module was written to optimize the control parameters and has already been used successfully.


#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "state.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "generated/airframe.h"
#include "firmwares/fixedwing/nav.h"
// #include "math/pprz_algebra_int.h"
// #include "math/pprz_algebra_float.h"

// For Downlink



float SquareSumErr_airspeed;
float SquareSumErr_altitude;
float SquareSumErr_position;
float ToleranceAispeed;
float ToleranceAltitude;
float TolerancePosition;
bool_t benchm_reset;
bool_t benchm_go;


//uint8_t numOfCount;



void flight_benchmark_init(void)
{
  SquareSumErr_airspeed = 0;
  SquareSumErr_altitude = 0;
  SquareSumErr_position = 0;
  ToleranceAispeed = BENCHMARK_TOLERANCE_AIRSPEED;
  ToleranceAltitude = BENCHMARK_TOLERANCE_ALTITUDE;
  TolerancePosition = BENCHMARK_TOLERANCE_POSITION;
  benchm_reset = 0;
  benchm_go = 0;
}

void flight_benchmark_periodic(void)
{
  float Err_airspeed = 0;
  float Err_altitude = 0;
  float Err_position = 0;

  if (benchm_reset) {
    flight_benchmark_reset();
    benchm_reset = 0;
  }

  if (benchm_go) {
#if USE_AIRSPEED && defined(BENCHMARK_AIRSPEED)
    Err_airspeed = fabs(*stateGetAirspeed_f() - v_ctl_auto_airspeed_setpoint);
    if (Err_airspeed > ToleranceAispeed) {
      Err_airspeed = Err_airspeed - ToleranceAispeed;
      SquareSumErr_airspeed += (Err_airspeed * Err_airspeed);
    }
#endif

#ifdef BENCHMARK_ALTITUDE
    Err_altitude = fabs(stateGetPositionUtm_f()->alt - v_ctl_altitude_setpoint);
    if (Err_altitude > ToleranceAltitude) {
      Err_altitude = Err_altitude - ToleranceAltitude;
      SquareSumErr_altitude += (Err_altitude * Err_altitude);
    }
#endif

#ifdef BENCHMARK_POSITION

    //---------------This part is a waste of memory and calculation power -  but it works - feel free to optimize it ;-) -----------------

    //  err_temp = waypoints[target].x - stateGetPositionEnu_f()->x;
    float deltaPlaneX = 0;
    float deltaPlaneY = 0;
    float Err_position_segment = 0;
    float Err_position_circle = 0;

//    if (nav_in_segment){
    float deltaX = nav_segment_x_2 - nav_segment_x_1;
    float deltaY = nav_segment_y_2 - nav_segment_y_1;
    float anglePath = atan2(deltaX, deltaY);
    deltaPlaneX = nav_segment_x_2 - stateGetPositionEnu_f()->x;
    deltaPlaneY = nav_segment_y_2 - stateGetPositionEnu_f()->y;
    float anglePlane = atan2(deltaPlaneX, deltaPlaneY);
    float angleDiff = fabs(anglePlane - anglePath);
    Err_position_segment = fabs(sin(angleDiff) * sqrt(deltaPlaneX * deltaPlaneX + deltaPlaneY * deltaPlaneY));
//    }

//    if (nav_in_circle){
    deltaPlaneX = nav_circle_x - stateGetPositionEnu_f()->x;
    deltaPlaneY = nav_circle_y - stateGetPositionEnu_f()->y;
    Err_position_circle = fabs(sqrt(deltaPlaneX * deltaPlaneX + deltaPlaneY * deltaPlaneY) - nav_circle_radius);
//    }
    if (Err_position_circle < Err_position_segment) {
      Err_position = Err_position_circle;
    } else {
      Err_position = Err_position_segment;
    }

    if (Err_position > TolerancePosition) {
      SquareSumErr_position += (Err_position * Err_position);
    }
#endif
  }

  DOWNLINK_SEND_FLIGHT_BENCHMARK(DefaultChannel, DefaultDevice, &SquareSumErr_airspeed, &SquareSumErr_altitude,
                                 &SquareSumErr_position, &Err_airspeed, &Err_altitude, &Err_position)

}

void flight_benchmark_reset(void)
{
  SquareSumErr_airspeed = 0;
  SquareSumErr_altitude = 0;
  SquareSumErr_position = 0;
}














