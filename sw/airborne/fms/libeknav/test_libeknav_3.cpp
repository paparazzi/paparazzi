
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"


#include <event.h>
extern "C" {
#include "std.h"
#include "fms/fms_debug.h"
#include "fms/fms_periodic.h"
#include "fms/fms_spi_link.h"
#include "fms/fms_autopilot_msg.h"
#include "booz/booz_imu.h"
  /* our sensors            */
  struct BoozImuFloat imu;
}

static void main_init(void);
static void main_periodic(int my_sig_num);
static void main_dialog_with_io_proc(void);


/* initial state */
Vector3d pos_0_ecef(1017.67e3, -5079.282e3, 3709.041e3);
Vector3d speed_0_ecef(0., 0., 0.);
//  Vector3d orientation(0., 0., 0.);
Vector3d bias_0(0., 0., 0.);

/* initial covariance */
const double pos_cov_0 =  1e2*1e2;
const double speed_cov_0 =  3.*3.;
//  const double orientation_cov_0 =  RadOfDeg(5.)*RadOfDeg(5.);
const double bias_cov_0 =  0.447;

/* system noise      */
const Vector3d gyro_white_noise = Vector3d::Ones()*0.1*0.1;
const Vector3d gyro_stability_noise = Vector3d::Ones()*0.00001;
const Vector3d accel_white_noise = Vector3d::Ones()* 0.04*0.04;

static basic_ins_qkf ins = basic_ins_qkf(pos_0_ecef, pos_cov_0, bias_cov_0, speed_cov_0,
					 gyro_white_noise, gyro_stability_noise, accel_white_noise);


// import most common Eigen types 
USING_PART_OF_NAMESPACE_EIGEN

int main(int, char *[]) {

  main_init();

  std::cout << "test libeknav 1" << std::endl;

  /* Enter our mainloop */
  event_dispatch();
  
  TRACE(TRACE_DEBUG, "%s", "leaving mainloop... goodbye!\n");

  return 0;

}


static void main_init(void) {

  TRACE(TRACE_DEBUG, "%s", "Starting initialization\n");

  /* Initalize our SPI link to IO processor */
  if (spi_link_init()) {
    TRACE(TRACE_ERROR, "%s", "failed to open SPI link \n");
    return;
  }
  
  /* Initalize the event library */
  event_init();
  
  /* Initalize our ô so accurate periodic timer */
  if (fms_periodic_init(main_periodic)) {
    TRACE(TRACE_ERROR, "%s", "failed to start periodic generator\n");
    return; 
  }
 
}


static void main_periodic(int my_sig_num __attribute__ ((unused))) {

  static uint32_t cnt;
  cnt++;

  main_dialog_with_io_proc();

  const double dt = 1./512.;
  Vector3d gyro(0., 0., 0.);
  Vector3d accelerometer(0., 0., 9.81);
  ins.predict(gyro, accelerometer, dt);

  if (cnt % 10 == 0) { /* update mag at 50Hz */
    Vector3d magnetometer = Vector3d::UnitZ();
    const double   mag_noise = std::pow(5 / 180.0 * M_PI, 2);
    ins.obs_vector(magnetometer, magnetometer, mag_noise);
  }
  if (cnt % 128 == 0) /* update gps at 4 Hz */ {
    const Vector3d gps_pos_noise = Vector3d::Ones()  *10*10;
    const Vector3d gps_speed_noise = Vector3d::Ones()*0.1*0.1;
    ins.obs_gps_pv_report(pos_0_ecef, speed_0_ecef, gps_pos_noise, gps_speed_noise);
  }

}


static void main_dialog_with_io_proc() {

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid; 
  
  //  for (uint8_t i=0; i<6; i++) msg_out.payload.msg_down.pwm_outputs_usecs[i] = otp.servos_outputs_usecs[i];
  
  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  
  struct AutopilotMessagePTUp *in = &msg_in.payload.msg_up; 
  RATES_FLOAT_OF_BFP(imu.gyro, in->gyro);
  ACCELS_FLOAT_OF_BFP(imu.accel, in->accel); 
  MAGS_FLOAT_OF_BFP(imu.mag, in->mag); 

}
