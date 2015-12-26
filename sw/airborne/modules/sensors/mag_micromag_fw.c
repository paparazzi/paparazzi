#include "modules/sensors/mag_micromag_fw.h"
#include "modules/sensors/mag_micromag_fw_hw.h"
#include "led.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

volatile uint8_t micromag_status;
volatile int16_t micromag_values[MM_NB_AXIS];


void micromag_periodic(void)
{

  static uint8_t cnt = 0;

  if (micromag_status == MM_IDLE) {
    //    uint8_t * tab = &cnt;
    //    DOWNLINK_SEND_DEBUG(1,tab);
    cnt = 0;
    MmSendReq();
  } else if (micromag_status ==  MM_GOT_EOC) {
    MmReadRes();
  } else if (micromag_status == MM_WAITING_EOC) {
    cnt++;
    if (cnt > 50) {cnt = 0; micromag_status = MM_IDLE;}
  }
}

void micromag_event(void)
{

  int32_t mx = micromag_values[0];
  int32_t my = micromag_values[1];
  int32_t mz = micromag_values[2];

  if (micromag_status == MM_DATA_AVAILABLE) {
    DOWNLINK_SEND_IMU_MAG_RAW(DefaultChannel, DefaultDevice,
                              &mx,
                              &my,
                              &mz);
    micromag_status = MM_IDLE;
  }
}

void micromag_init(void)
{

  micromag_hw_init();

  uint8_t i;
  for (i = 0; i < MM_NB_AXIS; i++) {
    micromag_values[i] = 0;
  }
  micromag_status = MM_IDLE;
}

void micromag_reset()
{
  micromag_status = MM_IDLE;
}

void micromag_read()
{
  if (micromag_status == MM_IDLE) {
    MmSendReq();
  } else if (micromag_status ==  MM_GOT_EOC) {
    MmReadRes();
  } else if (micromag_status ==  MM_DATA_AVAILABLE) {
    micromag_status = MM_IDLE;
  }
}

