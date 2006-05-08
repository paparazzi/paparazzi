
/** \file main_ap_2.c
 *  \brief Dummy file to ease the transistion to ARM code
 *
 */

#include "main_ap.h"
#include "init_hw.h"
#include "interrupt_hw.h"
#include "sys_time.h"
#include "led.h"
#include "modem.h"
#include "gps.h"
#include "print.h"

#include "spi.h"
#include "link_mcu.h"

#include "infrared.h"
#include "estimator.h"
#include "nav.h"

#include "airframe.h"
float pitch_of_vz;
float pitch_of_vz_pgain;
bool_t auto_pitch = FALSE;
uint8_t ac_ident = AC_ID;
float desired_roll = 0.;
float desired_pitch = 0.;
float desired_x = 0.;
float desired_y = 0.;
float desired_altitude;
float desired_climb;
uint8_t pprz_mode;
uint8_t vertical_mode;
uint8_t lateral_mode;
uint8_t horizontal_mode;
uint8_t inflight_calib_mode;
uint8_t mcu1_status;
uint8_t ir_estim_mode;
//int32_t nav_utm_east0;
//int32_t nav_utm_north0;
//int8_t nav_utm_zone0;
//float estimator_phi;
//float estimator_psi;
//float estimator_theta;
//float estimator_x;
//float estimator_y;
//float estimator_z;
//float estimator_hspeed_mod;
//float estimator_hspeed_dir;
//float estimator_z_dot;
//uint16_t estimator_flight_time;
//bool_t in_segment;
//int16_t segment_x_1;
//int16_t segment_y_1;
//int16_t segment_x_2;
//int16_t segment_y_2;
float energy;
int16_t desired_gaz;
uint8_t vsupply;
bool_t low_battery;
//uint16_t block_time;
//uint16_t stage_time;
//float climb_sum_err;
//float climb_pgain;
//float course_pgain;
//bool_t in_circle;
//int16_t circle_x;
//int16_t circle_y;
//int16_t circle_radius;
uint8_t modem_nb_ovrn;
uint8_t mcu1_ppm_cpt;

#include "ap_downlink.h"
#define PeriodicSendDlValue() {}

void init_ap( void ) {
  /* if AP is running in a separate MCU */
#ifndef SINGLE_MCU
  hw_init();
  sys_time_init();
#ifdef LED
  led_init();
#endif /* LED */
#ifdef USE_UART0
  Uart0Init();
#endif  /* USE_UART0 */
#if USE_UART1
  Uart1Init();
#endif
#ifdef ADC
  adc_init();
#endif
#endif  /* !SINGLE_MCU */

#ifdef MODEM
  modem_init();
#endif /* MODEM */
#ifdef GPS
  gps_init();
  gps_configure();
#endif
#if defined MCU_SPI_LINK
  spi_init();
  link_mcu_init();
#endif

  /* if AP is running in a separate MCU */
#ifndef SINGLE_MCU
 int_enable();
#endif /* FBW */
}

void periodic_task_ap( void ) {
#ifdef DOWNLINK
  PeriodicSendAp();
#endif

#if defined MCU_SPI_LINK
  uint8_t j;
  for (j=0; j<10; j++) {
    ((uint8_t*)&link_mcu_from_ap_msg)[j] = 42+j;
  }
  link_mcu_send();
#endif
  {
    extern volatile uint16_t adc0_val[];
    //    int_disable();
    DOWNLINK_SEND_ADC(&ac_ident, NB_ADC, adc0_val);
    //    int_enable();
  }
}

void event_task_ap( void ) {
#ifdef GPS
  if (GpsBuffer()) {
    ReadGpsBuffer();
  }
  if (gps_msg_received) {
    /* parse and use GPS messages */
    parse_gps_msg();
    gps_msg_received = FALSE;
    if (gps_pos_available) {
      //      LED_TOGGLE(2);
      use_gps_pos();
      gps_pos_available = FALSE;
    }
  }
#endif /* GPS */
#ifdef MCU_SPI_LINK
  if (spi_message_received) {
    /* Got a message on SPI. */
    spi_message_received = FALSE;
    link_mcu_event_task();
  }
#endif
 if (inter_mcu_received_fbw) {
   /* receive radio control task from fbw */
   inter_mcu_received_fbw = FALSE;
   DOWNLINK_SEND_DEBUG2((uint8_t)sizeof(struct link_mcu_msg), ((uint8_t*)&link_mcu_from_fbw_msg));
 }
}
