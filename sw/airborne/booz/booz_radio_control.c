#include "booz_radio_control.h"

#include "led.h"

struct RadioControl radio_control;

void radio_control_init(void) {
  radio_control.status = RADIO_CONTROL_REALLY_LOST;
  radio_control.time_since_last_frame = RADIO_CONTROL_REALLY_LOST_TIME;
  radio_control.frame_rate = 0;
  radio_control.frame_cpt = 0;
}


void radio_control_periodic(void) {
 
  /* compute frame rate */
  RunOnceEvery(60, {
      radio_control.frame_rate = radio_control.frame_cpt;
      radio_control.frame_cpt = 0;
    });
  
  /* check for timeouts */
  if (radio_control.time_since_last_frame >= RADIO_CONTROL_REALLY_LOST_TIME) {
    radio_control.status = RADIO_CONTROL_REALLY_LOST;
  } 
  else {
    if (radio_control.time_since_last_frame >= RADIO_CONTROL_LOST_TIME)
      radio_control.status = RADIO_CONTROL_LOST;
    radio_control.time_since_last_frame++;
  }

  /* sigal status with LEDs */
#if defined RADIO_CONTROL_LED
  if (radio_control.status == RADIO_CONTROL_OK) {
    LED_ON(RADIO_CONTROL_LED);
  }
  else {
    LED_OFF(RADIO_CONTROL_LED);
  }
#endif

}
