/*
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

#include <inttypes.h>
#include <string.h>

#define DATALINK_C

#include "test_board.h"

#include "std.h"
#include "mcu.h"
#include "mcu_periph/uart.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "led.h"

#include "subsystems/datalink/datalink.h"
#include "generated/settings.h"

#include "lisa/lisa_baro.h"
#include "actuators/actuators_pwm.h"

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void );

static void test_none_start(void);
static void test_none_periodic(void);
static void test_none_event(void);

static void test_baro_start(void);
static void test_baro_periodic(void);
static void test_baro_event(void);

static void test_bldc_start(void);
static void test_bldc_periodic(void);
static void test_bldc_event(void);

static void test_srvo_start(void);
static void test_srvo_periodic(void);
static void test_srvo_event(void);

static void test_uart_start(void);
static void test_uart_periodic(void);
static void test_uart_event(void);

static void all_led_green(void);
static void all_led_red(void);
static void all_led_off(void);

struct TestFuns {
  void (*_start)(void);
  void (*_periodic)(void);
  void (*_event)(void);
};

struct TestFuns tests[] = {
  {._start=test_none_start, ._periodic=test_none_periodic, ._event=test_none_event},
  {._start=test_baro_start, ._periodic=test_baro_periodic, ._event=test_baro_event},
  {._start=test_bldc_start, ._periodic=test_bldc_periodic, ._event=test_bldc_event},
  {._start=test_srvo_start, ._periodic=test_srvo_periodic, ._event=test_srvo_event},
  {._start=test_uart_start, ._periodic=test_uart_periodic, ._event=test_uart_event}
};

enum TestType cur_test;


int main( void ) {
  main_init();
  while(1) {
    if (sys_time_check_and_ack_timer(0))
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {

  mcu_init();
  sys_time_register_timer((1./PERIODIC_FREQUENCY), NULL);
  led_init();

  baro_init();
  actuators_init();

  //  cur_test = TestTypeNone;
  cur_test = TestTypeBldc;

}

static inline void main_periodic_task( void ) {

  LED_PERIODIC();
  RunOnceEvery(256, {DOWNLINK_SEND_ALIVE(DefaultChannel, DefaultDevice, 16, MD5SUM);});

  tests[cur_test]._periodic();

}

static inline void main_event_task( void ) {

  DatalinkEvent();

  tests[cur_test]._event();

}

void start_test(void) {
  all_led_off();
  tests[cur_test]._start();
}




/*
 *
 * Test nothing
 *
 */
static void test_none_start(void) {}
static void test_none_periodic(void) {}
static void test_none_event(void) {}


/*
 *
 * Test baro
 *
 */
static inline void test_baro_on_baro_diff(void);
static inline void test_baro_on_baro_abs(void);
static void test_baro_start(void) {all_led_green();}
static void test_baro_periodic(void) {
  RunOnceEvery(2, {baro_periodic();});
  RunOnceEvery(100,{
      uint16_t i2c2_queue_full_cnt        = i2c2.errors->queue_full_cnt;
      uint16_t i2c2_ack_fail_cnt          = i2c2.errors->ack_fail_cnt;
      uint16_t i2c2_miss_start_stop_cnt   = i2c2.errors->miss_start_stop_cnt;
      uint16_t i2c2_arb_lost_cnt          = i2c2.errors->arb_lost_cnt;
      uint16_t i2c2_over_under_cnt        = i2c2.errors->over_under_cnt;
      uint16_t i2c2_pec_recep_cnt         = i2c2.errors->pec_recep_cnt;
      uint16_t i2c2_timeout_tlow_cnt      = i2c2.errors->timeout_tlow_cnt;
      uint16_t i2c2_smbus_alert_cnt       = i2c2.errors->smbus_alert_cnt;
      uint16_t i2c2_unexpected_event_cnt  = i2c2.errors->unexpected_event_cnt;
      uint32_t i2c2_last_unexpected_event = i2c2.errors->last_unexpected_event;
      const uint8_t _bus2 = 2;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c2_queue_full_cnt,
                               &i2c2_ack_fail_cnt,
                               &i2c2_miss_start_stop_cnt,
                               &i2c2_arb_lost_cnt,
                               &i2c2_over_under_cnt,
                               &i2c2_pec_recep_cnt,
                               &i2c2_timeout_tlow_cnt,
                               &i2c2_smbus_alert_cnt,
                               &i2c2_unexpected_event_cnt,
                               &i2c2_last_unexpected_event,
                               &_bus2);
    });
}
static void test_baro_event(void) {BaroEvent(test_baro_on_baro_abs, test_baro_on_baro_diff);}
static inline void test_baro_on_baro_abs(void) {
  RunOnceEvery(5,{DOWNLINK_SEND_BOOZ_BARO2_RAW(DefaultChannel, DefaultDevice, &baro.abs_raw, &baro.diff_raw);});
}
static inline void test_baro_on_baro_diff(void) {
  RunOnceEvery(5,{DOWNLINK_SEND_BOOZ_BARO2_RAW(DefaultChannel, DefaultDevice, &baro.abs_raw, &baro.diff_raw);});
}


/*
 *
 * Test motor controller
 *
 */

static void test_bldc_start(void) {}
static void test_bldc_periodic(void) {

  i2c1_buf[0] = 0x04;
  i2c1_transmit(0x58, 1, NULL);

  RunOnceEvery(100,{
      uint16_t i2c1_queue_full_cnt        = i2c1.errors->queue_full_cnt;
      uint16_t i2c1_ack_fail_cnt          = i2c1.errors->ack_fail_cnt;
      uint16_t i2c1_miss_start_stop_cnt   = i2c1.errors->miss_start_stop_cnt;
      uint16_t i2c1_arb_lost_cnt          = i2c1.errors->arb_lost_cnt;
      uint16_t i2c1_over_under_cnt        = i2c1.errors->over_under_cnt;
      uint16_t i2c1_pec_recep_cnt         = i2c1.errors->pec_recep_cnt;
      uint16_t i2c1_timeout_tlow_cnt      = i2c1.errors->timeout_tlow_cnt;
      uint16_t i2c1_smbus_alert_cnt       = i2c1.errors->smbus_alert_cnt;
      uint16_t i2c1_unexpected_event_cnt  = i2c1.errors->unexpected_event_cnt;
      uint32_t i2c1_last_unexpected_event = i2c1.errors->last_unexpected_event;
      const uint8_t _bus1 = 1;
      DOWNLINK_SEND_I2C_ERRORS(DefaultChannel, DefaultDevice,
                               &i2c1_queue_full_cnt,
                               &i2c1_ack_fail_cnt,
                               &i2c1_miss_start_stop_cnt,
                               &i2c1_arb_lost_cnt,
                               &i2c1_over_under_cnt,
                               &i2c1_pec_recep_cnt,
                               &i2c1_timeout_tlow_cnt,
                               &i2c1_smbus_alert_cnt,
                               &i2c1_unexpected_event_cnt,
                               &i2c1_last_unexpected_event,
                               &_bus1);
    });
}

static void test_bldc_event(void) {}


/*
 *
 * Test servos
 *
 */
static void test_srvo_start(void) {}
static void test_srvo_periodic(void) {
  static float foo = 0.;
  foo += 0.0025;
  int32_t bar = 1500 + 500. * sin(foo);
  for (uint8_t i=0; i<6; i++)
    actuators_pwm_values[i] = bar;
  actuators_pwm_commit();
}
static void test_srvo_event(void) {}



/*
 *
 * Test Uarts
 *
 */
enum UartTestType { OneToThree, ThreeToOne};
static const uint8_t buf_src[] = { 42, 43, 44, 45, 46, 122, 126, 128 };
static       uint8_t buf_dest[sizeof(buf_src)];
static uint8_t idx_tx;
static uint8_t idx_rx;
static enum UartTestType direction;

static void test_uart_start(void) {
  idx_rx = 0;
  idx_tx = 0;
  direction = OneToThree;
}

static void test_uart_periodic(void) {

  if (idx_tx<sizeof(buf_src)) {
    switch (direction) {
      case OneToThree : uart_transmit(&uart1, buf_src[idx_tx]); break;
      case ThreeToOne : uart_transmit(&uart3, buf_src[idx_tx]); break;
      default: break;
    }
    idx_tx++;
  }

}

static void test_uart_event(void) {

  if (uart_char_available(&uart3)) {
    buf_dest[idx_rx] = uart_getch(&uart3);
    if (idx_rx<sizeof(buf_src)) {
      DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, sizeof(buf_src), buf_dest);
      idx_rx++;
      if (idx_rx == sizeof(buf_src)) {
    if ( memcmp(buf_dest, buf_src, sizeof(buf_src)) ) {
      all_led_red();  // test failed
    }
    else { // start test in other direction
        idx_rx = 0;
        idx_tx = 0;
        direction = ThreeToOne;
    }
      }
    }
  }

  if (uart_char_available(&uart1)) {
    buf_dest[idx_rx] = uart_getch(&uart1);
    if (idx_rx<sizeof(buf_src)) {
      DOWNLINK_SEND_DEBUG(DefaultChannel, DefaultDevice, sizeof(buf_src), buf_dest);
      idx_rx++;
      if (idx_rx == sizeof(buf_src)) {
    if ( memcmp(buf_dest, buf_src, sizeof(buf_src)) ) {
      all_led_red();
    }
    else {
      all_led_green();
    }
      }
    }
  }
}



/*
 *
 * LED utilities
 *
 */

static void all_led_green(void) {
  LED_OFF(0); LED_OFF(2); LED_OFF(4); LED_OFF(6);
  LED_ON(1);  LED_ON(3);  LED_ON(5);  LED_ON(7);
}

static void all_led_red(void) {
  LED_OFF(1); LED_OFF(3); LED_OFF(5); LED_OFF(7);
  LED_ON(0);  LED_ON(2);  LED_ON(4);  LED_ON(6);
}

static void all_led_off(void) {
  LED_OFF(0); LED_OFF(2); LED_OFF(4); LED_OFF(6);
  LED_OFF(1); LED_OFF(3); LED_OFF(5); LED_OFF(7);
}


/*
 *
 * Datalink
 *
 */
#define IdOfMsg(x) (x[1])

void dl_parse_msg(void) {

  uint8_t msg_id = IdOfMsg(dl_buffer);
  switch (msg_id) {

  case  DL_PING:
    {
      DOWNLINK_SEND_PONG(DefaultChannel, DefaultDevice);
    }
    break;

  case DL_SETTING :
    {
      if (DL_SETTING_ac_id(dl_buffer) != AC_ID) break;
      uint8_t i = DL_SETTING_index(dl_buffer);
      float var = DL_SETTING_value(dl_buffer);
      DlSetting(i, var);
      DOWNLINK_SEND_DL_VALUE(DefaultChannel, DefaultDevice, &i, &var);
    }
    break;

 default:
    break;
  }
}
