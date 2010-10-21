#include "dc.h"

uint8_t dc_timer;
uint8_t dc_periodic_shutter;
uint8_t dc_shutter_timer;
uint8_t dc_utm_threshold;
uint16_t dc_photo_nr = 0;

uint8_t dc_shoot = 0;


#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "uart.h"
#include "messages.h"
#include "downlink.h"


void dc_send_shot_position(void)
{
      int16_t phi = DegOfRad(estimator_phi*10.0f);
      int16_t theta = DegOfRad(estimator_theta*10.0f);
      float gps_z = ((float)gps_alt) / 100.0f;
      DOWNLINK_SEND_DC_SHOT(DefaultChannel, &dc_photo_nr, &gps_utm_east, &gps_utm_north, &gps_z, &gps_utm_zone, &phi, &theta,  &gps_course, &gps_gspeed, &gps_itow);
      dc_photo_nr++;	
}

uint8_t dc_shutter( void )
{
  dc_timer = SHUTTER_DELAY; 
  DC_PUSH(DC_SHUTTER_LED);
  dc_send_shot_position();

  return 0;
}

