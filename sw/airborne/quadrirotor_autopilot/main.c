#include <inttypes.h>
#include <avr/signal.h>
#include <avr/interrupt.h>

#include "messages.h"
#include "downlink.h"
#include "airframe.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "link_fbw.h"
#include "spi.h"
#include "autopilot.h"

#include "imu.h"
#include "kalman.h"
#include "control.h"



#define TRANS_RAW_IMU()       {	   \
	static uint8_t foo; foo++; \
	if (!(foo%10)) { \
	  DOWNLINK_SEND_RAW_IMU( &adc_samples[ADC_CHANNEL_GYRO_X], &adc_samples[ADC_CHANNEL_GYRO_Y], \
				 &adc_samples[ADC_CHANNEL_GYRO_Z], &adc_samples[ADC_CHANNEL_ACCEL_X],\
				 &adc_samples[ADC_CHANNEL_ACCEL_Y], &adc_samples[ADC_CHANNEL_ACCEL_Z]); \
	} \
      }\


#define TRANS_IMU()     { \
	static uint8_t foo; foo++; \
	if (!(foo%10)) { \
	  DOWNLINK_SEND_IMU( &imu_sample.gyro_x, &imu_sample.gyro_y, &imu_sample.gyro_z, \
			     &imu_sample.accel_x, &imu_sample.accel_y, &imu_sample.accel_z); \
	} \
      } \

#define TRANS_KALMAN()      {  \
	static uint8_t foo; foo++; \
	if (!(foo%10)) { \
	  DOWNLINK_SEND_KALMAN( &kalman_public.phi, &kalman_public.phi_dot,  \
				&kalman_public.gyro_phi_bias, &kalman_public.theta, \
				&kalman_public.theta_dot, &kalman_public.gyro_theta_bias);	\
	} \
      } \
    

#define PERIODIC_SEND_BAT() DOWNLINK_SEND_BAT(&vsupply, &estimator_flight_time, &low_battery)
#define PERIODIC_SEND_ATTITUDE() DOWNLINK_SEND_ATTITUDE( &(kalman_public.phi), &psi, &(kalman_public.theta))
#define PERIODIC_SEND_ADC() {}
#define PERIODIC_SEND_SETTINGS() {}
#define PERIODIC_SEND_DESIRED() {}
#define PERIODIC_SEND_CLIMB_PID() {}
#define PERIODIC_SEND_PPRZ_MODE() DOWNLINK_SEND_ATTITUDE( &(kalman_public.phi), &psi, &(kalman_public.theta))
#define PERIODIC_SEND_DEBUG() {}
#define PERIODIC_SEND_NAVIGATION_REF() {} 


uint8_t fatal_error_nb = 0;
uint16_t cputime = 0;
float psi = 0.;

uint8_t mcu1_status;
uint8_t pprz_mode;
uint8_t vertical_mode;
uint8_t inflight_calib_mode;
uint8_t ir_estim_mode;

uint8_t vsupply;

uint8_t low_battery = FALSE;
uint16_t estimator_flight_time = 0;

inline void copy_from_to_fbw ( void ) {
  to_fbw.channels[RADIO_THROTTLE] = from_fbw.channels[RADIO_THROTTLE];
  //  to_fbw.channels[RADIO_ROLL] = from_fbw.channels[RADIO_ROLL];
  //  to_fbw.channels[RADIO_PITCH] = from_fbw.channels[RADIO_PITCH];
  to_fbw.channels[RADIO_YAW] = from_fbw.channels[RADIO_YAW];
  to_fbw.status = 0;
}

inline uint8_t pprz_mode_update( void ) {
  ModeUpdate(pprz_mode, PPRZ_MODE_OF_PULSE(from_fbw.channels[RADIO_MODE], from_fbw.status));
}

inline uint8_t mcu1_status_update( void ) {
  uint8_t new_mode = from_fbw.status;
  if (mcu1_status != new_mode) {
    bool_t changed = ((mcu1_status&MASK_FBW_CHANGED) != (new_mode&MASK_FBW_CHANGED));
    mcu1_status = new_mode;
    return changed;
  }
  return FALSE;
}

inline void radio_control_task( void ) {
  if (link_fbw_receive_valid) {
    uint8_t mode_changed = FALSE;
    copy_from_to_fbw();
    if (bit_is_set(from_fbw.status, AVERAGED_CHANNELS_SENT)) {
      bool_t pprz_mode_changed = pprz_mode_update();
      mode_changed |= pprz_mode_changed;
    }
    mode_changed |= mcu1_status_update();
    if ( mode_changed )
      DOWNLINK_SEND_PPRZ_MODE(&pprz_mode, &vertical_mode, &inflight_calib_mode, &mcu1_status, &ir_estim_mode);
    
    if (pprz_mode == PPRZ_MODE_AUTO1) {
      control_desired_phi_dot =  FLOAT_OF_PPRZ(from_fbw.channels[RADIO_ROLL], 0., -1.25);
      control_desired_theta_dot =  FLOAT_OF_PPRZ(from_fbw.channels[RADIO_PITCH], 0., 1.25);
    }
    else if (pprz_mode == PPRZ_MODE_AUTO2) {
      control_desired_phi =  FLOAT_OF_PPRZ(from_fbw.channels[RADIO_ROLL], 0., -0.6);
      control_desired_theta =  FLOAT_OF_PPRZ(from_fbw.channels[RADIO_PITCH], 0., 0.6);
    }
    vsupply = from_fbw.vsupply;
  }
}

inline void periodic_task ( void ) {
  static uint8_t _20Hz   = 0;
  
  _20Hz++;
  if (_20Hz>=3) _20Hz=0;

  imu_update( );
  TRANS_RAW_IMU();
  TRANS_IMU();
  kalman_state_update(imu_sample.gyro_x, imu_sample.gyro_y);
  kalman_kalman_update(imu_sample.accel_x, imu_sample.accel_y, imu_sample.accel_z);
  TRANS_KALMAN();
  if (pprz_mode == PPRZ_MODE_AUTO2) {
    //    control_run_attitude_loop();
    control_run_raw_rotational_speed_loop();
    to_fbw.channels[RADIO_ROLL] = control_command_roll;
    to_fbw.channels[RADIO_PITCH] = control_command_pitch;
  }
  if (pprz_mode == PPRZ_MODE_AUTO1 || pprz_mode == PPRZ_MODE_AUTO2) {
    control_run_rotational_speed_loop();
    to_fbw.channels[RADIO_ROLL] = control_command_roll;
    to_fbw.channels[RADIO_PITCH] = control_command_pitch;
  }
  link_fbw_send();  
  if (!_20Hz)
    PeriodicSend();
}

int main(void) {

  timer_init();
  modem_init();
  adc_init();
  spi_init();
  link_fbw_init();
  //  uart0_init();
  //  imu_init();
  sei();
  
  while (1) {
      if (timer_periodic())
      periodic_task();
    if (link_fbw_receive_complete) {
      radio_control_task();
      link_fbw_receive_complete = FALSE;
    }
  }
  return 0;
}



