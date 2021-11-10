/*
  C code to connect a CHIMU using uart
*/


#include <stdbool.h>

// SPI
#include "mcu_periph/spi.h"
#include "mcu_periph/spi_slave_hs_arch.h"

// Output
#include "state.h"

// For centripedal corrections
#include "modules/gps/gps.h"
#include "subsystems/ahrs.h"

#include "generated/airframe.h"

#if CHIMU_DOWNLINK_IMMEDIATE
#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"
#endif

#include "ins_module.h"
#include "imu_chimu.h"
#include "ahrs_chimu.h"

#include "led.h"

CHIMU_PARSER_DATA CHIMU_DATA;

INS_FORMAT ins_roll_neutral;
INS_FORMAT ins_pitch_neutral;

struct AhrsChimu ahrs_chimu;

void ahrs_chimu_update_gps(uint8_t gps_fix, uint16_t gps_speed_3d);

#include "modules/core/abi.h"
/** ABI binding for gps data.
 * Used for GPS ABI messages.
 */
#ifndef AHRS_CHIMU_GPS_ID
#define AHRS_CHIMU_GPS_ID GPS_MULTI_ID
#endif
PRINT_CONFIG_VAR(AHRS_CHIMU_GPS_ID)
static abi_event gps_ev;
static void gps_cb(uint8_t sender_id __attribute__((unused)),
                   uint32_t stamp __attribute__((unused)),
                   struct GpsState *gps_s)
{
  ahrs_chimu_update_gps(gps_s->fix, gps_s->speed_3d);
}

static bool ahrs_chimu_enable_output(bool enable)
{
  ahrs_chimu.is_enabled = enable;
  return ahrs_chimu.is_enabled;
}

void ahrs_chimu_register(void)
{
  ahrs_chimu_init();
  ahrs_register_impl(ahrs_chimu_enable_output);
  AbiBindMsgGPS(AHRS_CHIMU_GPS_ID, &gps_ev, gps_cb);
}

void ahrs_chimu_init(void)
{
  ahrs_chimu.is_enabled = true;
  ahrs_chimu.is_aligned = false;

  // uint8_t ping[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG00_PING, 0x00, 0xE6 };
  uint8_t rate[12] = {CHIMU_STX, CHIMU_STX, 0x06, CHIMU_BROADCAST, MSG10_UARTSETTINGS, 0x05, 0xff, 0x79, 0x00, 0x00, 0x01, 0x76 };  // 50Hz attitude only + SPI
  uint8_t quaternions[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG09_ESTIMATOR, 0x01, 0x39 }; // 25Hz attitude only + SPI
  // uint8_t rate[12] = {CHIMU_STX, CHIMU_STX, 0x06, CHIMU_BROADCAST, MSG10_UARTSETTINGS, 0x04, 0xff, 0x79, 0x00, 0x00, 0x01, 0xd3 }; // 25Hz attitude only + SPI
  // uint8_t euler[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG09_ESTIMATOR, 0x00, 0xaf }; // 25Hz attitude only + SPI

  new_ins_attitude = 0;

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

  // Init
  CHIMU_Init(&CHIMU_DATA);

  // Quat Filter
  CHIMU_Checksum(quaternions, 7);
  InsSend(quaternions, 7);

  // Wait a bit (SPI send zero)
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);

  // 50Hz data: attitude only
  CHIMU_Checksum(rate, 12);
  InsSend(rate, 12);
}


void parse_ins_msg(void)
{
  struct link_device *dev = InsLinkDevice;
  while (dev->char_available(dev->periph)) {
    uint8_t ch = dev->get_byte(dev->periph);

    if (CHIMU_Parse(ch, 0, &CHIMU_DATA)) {
      RunOnceEvery(25, LED_TOGGLE(3));
      if (CHIMU_DATA.m_MsgID == CHIMU_Msg_3_IMU_Attitude) {
        // TODO: remove this flag as it doesn't work with multiple AHRS
        new_ins_attitude = 1;
        if (CHIMU_DATA.m_attitude.euler.phi > M_PI) {
          CHIMU_DATA.m_attitude.euler.phi -= 2 * M_PI;
        }

        //FIXME
        ahrs_chimu.is_aligned = true;

        if (ahrs_chimu.is_enabled) {
          struct FloatEulers att = {
            CHIMU_DATA.m_attitude.euler.phi,
            CHIMU_DATA.m_attitude.euler.theta,
            CHIMU_DATA.m_attitude.euler.psi
          };
          stateSetNedToBodyEulers_f(&att);
          struct FloatRates rates = {
            CHIMU_DATA.m_sensor.rate[0],
            CHIMU_DATA.m_attrates.euler.theta,
            0.
          }; // FIXME rate r
          stateSetBodyRates_f(&rates);
        }
      } else if (CHIMU_DATA.m_MsgID == 0x02) {
#if CHIMU_DOWNLINK_IMMEDIATE
        RunOnceEvery(25, DOWNLINK_SEND_AHRS_EULER(DefaultChannel, DefaultDevice, &CHIMU_DATA.m_sensor.rate[0],
                     &CHIMU_DATA.m_sensor.rate[1], &CHIMU_DATA.m_sensor.rate[2]));
#endif
      }
    }
  }
}


void ahrs_chimu_update_gps(uint8_t gps_fix __attribute__((unused)), uint16_t gps_speed_3d)
{
  // Send SW Centripetal Corrections
  uint8_t centripedal[19] = {0xae, 0xae, 0x0d, 0xaa, 0x0b, 0x02,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00,   0xc2 };

  float gps_speed = 0;

  if (GpsFixValid()) {
    gps_speed = gps_speed_3d / 100.;
  }
  gps_speed = FloatSwap(gps_speed);

  memmove(&centripedal[6], &gps_speed, 4);

  // Fill X-speed

  CHIMU_Checksum(centripedal, 19);
  InsSend(centripedal, 19);

  // Downlink Send
}
