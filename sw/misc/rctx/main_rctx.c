
    /*

  - 0x101..0x10F reserved for rc transmitters
  - rc transmitters broadcast their messages (dest = 0xFFFF)
  - aircrafts are configured (conf.xml) to one rc transmitter (e.g. 0x102)
  - md5 sum be broadcasted?
  - rc tx receives md5 sum messages and displays that on LEDs
  - rc tx is switchable in addresses by button

  - XBee-message

       ID is AC_ID for aircraft, 0x100 for ground station

 1     A XBEE_START (0x7E)
 2     B LENGTH_MSB (A->E)
 3     C LENGTH_LSB
         XBEE_PAYLOAD
 4       0 XBEE_TX16 (0x01) / XBEE_RX16 (0x81)
 5       1 FRAME_ID (0)     / SRC_ID_MSB
 6       2 DEST_ID_MSB      / SRC_ID_LSB
 7       3 DEST_ID_LSB      / XBEE_RSSI
 8       4 TX16_OPTIONS (0) / RX16_OPTIONS
           PPRZ_DATA
 9         0 SENDER_ID
10         1 MSG_ID
             MSG_PAYLOAD
11           0 RCTX_MODE
12           1 THOTTLE_LSB
13           2 THOTTLE_MSB
14           3 ROLL_LSB
15           4 ROLL_MSB
16           5 PITCH_LSB
17           6 PITCH_MSB
18     E XBEE_CHECKSUM (sum[A->D])

  - messages.xml

  <message name="RC_3CH" ID="27">
    <field name="mode"     type="uint8" unit="byte_mask"></field>
    <field name="throttle" type="int16" unit="pprz" format="%d"/>
    <field name="roll"     type="int16" unit="pprz" format="%d"/>
    <field name="pitch"    type="int16" unit="pprz" format="%d"/>
  </message>

  - xbee.h

#ifdef USE_DOWNLINK_BROADCAST
#define GROUND_STATION_ADDR 0xFFFF
#else
#define GROUND_STATION_ADDR 0x0100
#endif

  - datalink.c

#ifdef USE_RC_TELEMETRY
    if (msg_id == DL_RC_3CH && DL_RC_3CH_ac_id(dl_buffer) == TX_ID) {
LED_TOGGLE(3);
        bla_throttle = DL_RC_3CH_throttle(dl_buffer);
        bla_roll = DL_RC_3CH_roll(dl_buffer);
        bla_pitch = DL_RC_3CH_pitch(dl_buffer);
    } else
#endif // USE_RC_TELEMETRY

*/

#include <stdio.h>
#include "std.h"
#include "init_hw.h"
#include "sys_time.h"
#include "adc.h"
#include "led.h"
#include "interrupt_hw.h"
#include "uart_hw.h"
#include "uart.h"
#include "autopilot.h"
#include "modules/datalink/datalink.h"

#include "messages.h"
#include "modules/datalink/downlink.h"

#if ((AC_ID > 0x108) || (AC_ID < (0x101)))
//#error aircraft ID should be 0x101..0x108 for RC transmitter
#endif

/** Maximum time allowed for low battery level */
#define LOW_BATTERY_DELAY 5

#ifdef ADC
struct adc_buf vsupply_adc_buf;
#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) DefaultVoltageOfAdc(adc)
#endif
#endif

#define LOW_BATTERY_DECIVOLT (CATASTROPHIC_BAT_LEVEL*10)

uint16_t rctx_vsupply_decivolt;
uint8_t rctx_under_voltage;
uint8_t rctx_mode;

void init_rctx( void );
void event_task_rctx( void);
void periodic_task_rctx( void );

// datalink.c
#define SenderIdOfMsg(x) (x[0])
#define IdOfMsg(x) (x[1])

uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));
bool dl_msg_available;
uint16_t datalink_time;

void dl_parse_msg(void) {
  datalink_time = 0;
  uint8_t msg_id = IdOfMsg(dl_buffer);

  if (msg_id == DL_PING) {
    DOWNLINK_SEND_PONG();
  } else
  { /* Last else */ }
}


/********** MAIN *************************************************************/
int main( void ) {
  init_rctx();

  while(1) {
    if (sys_time_periodic())
      periodic_task_rctx();
    event_task_rctx();
  }
  return 0;
}

/********** INIT *************************************************************/
void init_rctx( void ) {
  hw_init();
  sys_time_init();
#ifdef LED
  led_init();
#endif
#ifdef USE_UART1
    Uart1Init();
#endif
#ifdef ADC
  adc_init();
  adc_buf_channel(ADC_CHANNEL_VSUPPLY, &vsupply_adc_buf, DEFAULT_AV_NB_SAMPLE);
#endif
#ifdef RADIO_CONTROL
  ppm_init();
#endif
  int_enable();

  /** - wait 0.5s (for modem init ?) */
  uint8_t init_cpt = 30;
  while (init_cpt) {
    if (sys_time_periodic())
      init_cpt--;
  }

#if defined DATALINK
#if DATALINK == XBEE
  xbee_init();
#endif
#endif /* DATALINK */
}

/********** EVENT ************************************************************/
void event_task_rctx( void) {
#ifdef RADIO_CONTROL
  if (ppm_valid) {
    ppm_valid = FALSE;
    radio_control_event_task();

#ifdef USE_RCTX_MODE_SWITCH
    // TODO: set rxtx_mode from GPIO connected switch (e.g. I2C pins)
#else
    rctx_mode = AP_MODE_OF_PULSE(rc_values[RADIO_MODE], 0) & 3;
#endif

    rctx_mode |= rctx_under_voltage << 2;
LED_TOGGLE(3);

    if (1)
    // TODO: check XBee busy pin
    // TODO: send only if aircraft is listening
    // TODO: send (here) only in auto1 and manual
    {
      DOWNLINK_SEND_RC_3CH(
            &rctx_mode,
            &rc_values[RADIO_THROTTLE],
            &rc_values[RADIO_ROLL],
            &rc_values[RADIO_PITCH]);
    }
  }
#endif

#if defined DATALINK

#if DATALINK == XBEE
  if (XBeeBuffer()) {
    ReadXBeeBuffer();
    if (xbee_msg_received) {
      xbee_parse_payload();
      xbee_msg_received = FALSE;
    }
  }
#endif

  if (dl_msg_available) {
    dl_parse_msg();
    dl_msg_available = FALSE;
  }
#endif /* DATALINK */
}

/************* PERIODIC ******************************************************/
void periodic_task_rctx( void ) {
  static uint8_t _10Hz = 0;
  static uint8_t _1Hz = 0;

  _10Hz++;
  if (_10Hz >= 6) _10Hz = 0;
  _1Hz++;
  if (_1Hz>=60) _1Hz=0;

#ifdef RADIO_CONTROL
  radio_control_periodic_task();
#endif

#ifdef DOWNLINK
// TODO: needed?  fbw_downlink_periodic_task();
#endif

#ifdef ADC
  if (!_10Hz)
    rctx_vsupply_decivolt = VoltageOfAdc((10*(vsupply_adc_buf.sum/vsupply_adc_buf.av_nb_sample)));

  if (!_1Hz) {
    static uint8_t t = 0;
    if (rctx_vsupply_decivolt < LOW_BATTERY_DECIVOLT) {
      t++;
    } else {
      t = 0;
      rctx_under_voltage = 0;
    }
    if (t >= LOW_BATTERY_DELAY) {
      LED_TOGGLE(1);
      rctx_under_voltage = 1;
    }

    if (0)
    // TODO: send (here) only in auto2
    {
      DOWNLINK_SEND_RC_3CH(
            &rctx_mode,
            &rc_values[RADIO_THROTTLE],
            &rc_values[RADIO_ROLL],
            &rc_values[RADIO_PITCH]);
    }
  }
#endif
}
