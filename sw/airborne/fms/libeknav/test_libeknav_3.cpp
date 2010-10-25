
#include <iostream>
#include <iomanip>

#include <Eigen/Core>

#include "ins_qkf.hpp"
#include <stdint.h>


#include <event.h>
extern "C" {
#include <unistd.h>
#include <time.h>
#include "std.h"
#include "fms/fms_debug.h"
#include "fms/fms_periodic.h"
#include "fms/fms_spi_link.h"
#include "fms/fms_autopilot_msg.h"
#include "subsystems/imu.h"
#include "fms/libeknav/raw_log.h"
  /* our sensors            */
  struct ImuFloat imu_float;
  /* raw log */
  static int raw_log_fd;
}

static void main_trick_libevent(void);
static void on_foo_event(int fd, short event __attribute__((unused)), void *arg);
static struct event foo_event;

#include "math/pprz_algebra_float.h"
static void main_rawlog_init(const char* filename);
static void main_rawlog_dump(void);

static void main_init(void);
static void main_periodic(int my_sig_num);
static void main_dialog_with_io_proc(void);
static void main_run_ins(void);


/* time measurement */
struct timespec start;

float absTime(struct timespec T){
	return (float)(T.tv_sec + T.tv_nsec*1e-9);
}

struct timespec time_diff(struct timespec end, struct timespec start){
	float difference = absTime(end)-absTime(start);
	struct timespec dT;
	dT.tv_sec = (int)difference;
	dT.tv_nsec = (difference-dT.tv_sec)*1000000000;
	return dT;
}
 
#define TIMER CLOCK_MONOTONIC 



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

  std::cout << "test libeknav 3" << std::endl;
  clock_gettime(TIMER, &start);
  main_init();
  /* add dev/null as event source so that libevent doesn't die */
  main_trick_libevent();
  
  
  TRACE(TRACE_DEBUG, "%s", "Entering mainloop\n");
  
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
   
  main_rawlog_init("/tmp/log_test3.bin");

}


static void main_periodic(int my_sig_num __attribute__ ((unused))) {

  main_dialog_with_io_proc();
  //  main_run_ins();
  main_rawlog_dump();

}


static void main_dialog_with_io_proc() {

  struct AutopilotMessageCRCFrame msg_in;
  struct AutopilotMessageCRCFrame msg_out;
  uint8_t crc_valid; 
  
  //  for (uint8_t i=0; i<6; i++) msg_out.payload.msg_down.pwm_outputs_usecs[i] = otp.servos_outputs_usecs[i];
  
  spi_link_send(&msg_out, sizeof(struct AutopilotMessageCRCFrame), &msg_in, &crc_valid);
  
  struct AutopilotMessagePTUp *in = &msg_in.payload.msg_up; 
  RATES_FLOAT_OF_BFP(imu_float.gyro, in->gyro);
  ACCELS_FLOAT_OF_BFP(imu_float.accel, in->accel); 
  MAGS_FLOAT_OF_BFP(imu_float.mag, in->mag); 

  {
    static uint32_t foo=0;
    foo++;
    if (!(foo%100))
      printf("%f %f %f\n",imu_float.gyro.p,imu_float.gyro.q,imu_float.gyro.r); 
    
  }

}

static void main_run_ins() {

  static uint32_t cnt;
  cnt++;

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




#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

static void main_trick_libevent(void) {

  int fd = open("/dev/ttyS0", O_RDONLY);
  if (fd == -1) {
    TRACE(TRACE_ERROR, "%s", "failed to open /dev/null \n");
    return;
  }
  event_set(&foo_event, fd, EV_READ | EV_PERSIST, on_foo_event, NULL);
  event_add(&foo_event, NULL);

}

static void on_foo_event(int fd __attribute__((unused)), short event __attribute__((unused)), void *arg __attribute__((unused))) {

} 





static void main_rawlog_init(const char* filename) {
  
  raw_log_fd = open(filename, O_WRONLY|O_CREAT, 00644);
  if (raw_log_fd == -1) {
    TRACE(TRACE_ERROR, "failed to open rawlog outfile (%s)\n", filename);
    return;
  }
}

static void main_rawlog_dump(void) {
  struct timespec now;
  clock_gettime(TIMER, &now);
  struct raw_log_entry e;
  
  e.time = absTime(time_diff(now, start));
  RATES_COPY(e.gyro, imu_float.gyro);
  VECT3_COPY(e.accel, imu_float.accel);
  VECT3_COPY(e.mag, imu_float.mag);
  write(raw_log_fd, &e, sizeof(e));

}



