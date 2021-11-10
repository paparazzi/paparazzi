/*
  C code to connect a CHIMU using uart
*/


#include <stdbool.h>

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

#include "led.h"

CHIMU_PARSER_DATA CHIMU_DATA;

INS_FORMAT ins_roll_neutral;
INS_FORMAT ins_pitch_neutral;

struct AhrsChimu ahrs_chimu;

static bool ahrs_chimu_enable_output(bool enable)
{
  ahrs_chimu.is_enabled = enable;
  return ahrs_chimu.is_enabled;
}

void ahrs_chimu_register(void)
{
  ahrs_chimu_init();
  ahrs_register_impl(ahrs_chimu_enable_output);
}

void ahrs_chimu_init(void)
{
  ahrs_chimu.is_enabled = true;
  ahrs_chimu.is_aligned = false;

  uint8_t ping[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG00_PING, 0x00, 0xE6 };
  uint8_t rate[12] = {CHIMU_STX, CHIMU_STX, 0x06, CHIMU_BROADCAST, MSG10_UARTSETTINGS, 0x05, 0xff, 0x79, 0x00, 0x00, 0x01, 0x76 };  // 50Hz attitude only + SPI
  uint8_t quaternions[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG09_ESTIMATOR, 0x01, 0x39 }; // 25Hz attitude only + SPI
  //  uint8_t rate[12] = {CHIMU_STX, CHIMU_STX, 0x06, CHIMU_BROADCAST, MSG10_UARTSETTINGS, 0x04, 0xff, 0x79, 0x00, 0x00, 0x01, 0xd3 }; // 25Hz attitude only + SPI
  //  uint8_t euler[7] = {CHIMU_STX, CHIMU_STX, 0x01, CHIMU_BROADCAST, MSG09_ESTIMATOR, 0x00, 0xaf }; // 25Hz attitude only + SPI

  new_ins_attitude = 0;

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;

  CHIMU_Init(&CHIMU_DATA);

  // Request Software version
  for (int i = 0; i < 7; i++) {
    InsUartSend1(ping[i]);
  }

  // Quat Filter
  for (int i = 0; i < 7; i++) {
    InsUartSend1(quaternions[i]);
  }

  // 50Hz
  CHIMU_Checksum(rate, 12);
  InsSend(rate, 12);
}


void parse_ins_msg(void)
{
  struct link_device *dev = InsLinkDevice;
  while (dev->char_available(dev->periph)) {
    uint8_t ch = dev->get_byte(dev->periph);

    if (CHIMU_Parse(ch, 0, &CHIMU_DATA)) {
      if (CHIMU_DATA.m_MsgID == 0x03) {
        new_ins_attitude = 1;
        RunOnceEvery(25, LED_TOGGLE(3));
        if (CHIMU_DATA.m_attitude.euler.phi > M_PI) {
          CHIMU_DATA.m_attitude.euler.phi -= 2 * M_PI;
        }

        ahrs_chimu.is_aligned = true;

        if (ahrs_chimu.is_enabled) {
          struct FloatEulers att = {
            CHIMU_DATA.m_attitude.euler.phi,
            CHIMU_DATA.m_attitude.euler.theta,
            CHIMU_DATA.m_attitude.euler.psi
          };
          stateSetNedToBodyEulers_f(&att);
        }

#if CHIMU_DOWNLINK_IMMEDIATE
        static uint8_t ahrs_chimu_id = AHRS_COMP_ID_CHIMU;
        DOWNLINK_SEND_AHRS_EULER(DefaultChannel, DefaultDevice,
                                 &CHIMU_DATA.m_attitude.euler.phi,
                                 &CHIMU_DATA.m_attitude.euler.theta,
                                 &CHIMU_DATA.m_attitude.euler.psi,
                                 &ahrs_chimu_id);
#endif

      }
    }
  }
}
