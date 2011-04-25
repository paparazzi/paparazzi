/*
C code to connect a CHIMU using uart
*/


#include <stdbool.h>

// SPI
#include "mcu_periph/spi.h"
#include "mcu_periph/spi_slave_hs_arch.h"

// Output
#include "estimator.h"

// For centripedal corrections
#include "gps.h"

// Telemetry
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

#include "ins_module.h"
#include "imu_chimu.h"

CHIMU_PARSER_DATA CHIMU_DATA;

INS_FORMAT ins_roll_neutral;
INS_FORMAT ins_pitch_neutral;

volatile uint8_t new_ins_attitude;

void ins_init( void ) 
{
  uint8_t rate[12] = {0xae, 0xae, 0x06, 0xaa, 0x10, 0x05, 0xff, 0x79, 0x00, 0x00, 0x01, 0x76 };	// 50Hz attitude only + SPI
//  uint8_t rate[12] = {0xae, 0xae, 0x06, 0xaa, 0x10, 0x04, 0xff, 0x79, 0x00, 0x00, 0x01, 0xd3 }; // 25Hz attitude only + SPI
//  uint8_t euler[7] = {0xae, 0xae, 0x01, 0xaa, 0x09, 0x00, 0xaf }; // 25Hz attitude only + SPI
  uint8_t quaternions[7] = {0xae, 0xae, 0x01, 0xaa, 0x09, 0x01, 0x39 }; // 25Hz attitude only + SPI
  
  CHIMU_Checksum(rate,12);
  
  new_ins_attitude = 0;
  
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  
  CHIMU_Init(&CHIMU_DATA);  
  
  // Quat Filter
  for (int i=0;i<7;i++)
  {
    InsSend1(quaternions[i]);
  }

  // Wait a bit (SPI send zero)
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);
  InsSend1(0);

  // 50Hz data: attitude only
  for (int i=0;i<12;i++)
  {
    InsSend1(rate[i]);
  }
  
}

void parse_ins_msg( void )
{
  while (InsLink(ChAvailable()))
  {
    uint8_t ch = InsLink(Getch());
    
    if (CHIMU_Parse(ch, 0, &CHIMU_DATA))
    {
      if(CHIMU_DATA.m_MsgID==0x03)
      {
	new_ins_attitude = 1;
	RunOnceEvery(25, LED_TOGGLE(3) );
	if (CHIMU_DATA.m_attitude.euler.phi > M_PI)
	{
	  CHIMU_DATA.m_attitude.euler.phi -= 2 * M_PI;
	}
	
	EstimatorSetAtt(CHIMU_DATA.m_attitude.euler.phi, CHIMU_DATA.m_attitude.euler.psi, CHIMU_DATA.m_attitude.euler.theta);
	EstimatorSetRate(CHIMU_DATA.m_sensor.rate[0],CHIMU_DATA.m_attrates.euler.theta);
      }
      else if(CHIMU_DATA.m_MsgID==0x02)
      {
	
	RunOnceEvery(25,DOWNLINK_SEND_AHRS_EULER(DefaultChannel, &CHIMU_DATA.m_sensor.rate[0], &CHIMU_DATA.m_sensor.rate[1], &CHIMU_DATA.m_sensor.rate[2]));
	
      }
    }
  }
}


//Frequency defined in conf *.xml
void ins_periodic_task( void ) 
{
  // Send SW Centripetal Corrections
  uint8_t centripedal[19] = {0xae, 0xae, 0x0d, 0xaa, 0x0b, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc2 };
  
  // Fill X-speed
  
  CHIMU_Checksum(centripedal,19);
  
  for (int i=0;i<19;i++)
  {
    InsSend1(centripedal[i]);
  }

  // Downlink Send
}

