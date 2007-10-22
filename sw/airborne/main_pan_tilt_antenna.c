#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart.h"

#include "messages.h"
#include "downlink.h"

#include "pt_ant_motors.h"
#include "pt_ant_sensors.h"
#include "pt_ant_estimator.h"
#include "gps.h"

#include "datalink.h"
//#include "traffic_info.h"

#include "i2c.h"
#include "AMI601.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

int main( void ) {
  main_init();
  while(1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  Uart0Init();
  Uart1Init();
  //  gps_init();
  pt_ant_motors_init();
  //  pt_ant_sensors_init_spi();
  //  pt_ant_sensors_init();

  i2c_init();
  ami601_init();

  int_enable();

  //  gps_configure_uart();

}

static inline void main_periodic_task( void ) {
  //  PtAntSensorsPeriodic();

#ifdef USE_GPS
  GpsPeriodic();
#endif


  DOWNLINK_SEND_ESTIMATOR(&pt_ant_motors_y_power, &pt_ant_motors_z_power);
  //  LED_TOGGLE(1);

  ami601_periodic();
  DOWNLINK_SEND_IMU_ACCEL_RAW(&ami601_val[3], &ami601_val[5], &ami601_val[1]); // accel ??
  DOWNLINK_SEND_IMU_GYRO_RAW(&ami601_val[0], &ami601_val[4], &ami601_val[2]);  // mag ??
  //  ami601_scale_measures();
  //  DOWNLINK_SEND_IMU_ACCEL(&ami601_ax, &ami601_ay, &ami601_az);
  //  DOWNLINK_SEND_IMU_GYRO(&ami601_mx, &ami601_my, &ami601_mz);

}

static inline void main_event_task( void ) {

  //PtAntSensorsEventCheckAndHandle();
  
  DlEventCheckAndHandle();

#ifdef USE_GPS
  if (GpsEventCheckAndHandle())
    return;
#endif  
}
