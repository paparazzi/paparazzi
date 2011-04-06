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
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  new_ins_attitude= 0;
  
  CHIMU_Init(&CHIMU_DATA);  
    
}

#define INS_LINK	SpiSlave

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
	// RunOnceEvery(25, LED_TOGGLE(3) );
	// LED_TOGGLE(3);
	if (CHIMU_DATA.m_attitude.euler.phi > M_PI)
	{
	  CHIMU_DATA.m_attitude.euler.phi -= 2 * M_PI;
	}
	/*
	if (CHIMU_DATA.m_attitude.euler.phi == tempang)
	{
	  LED_ON(3);
	}
	else
	{
	  LED_OFF(3);
	}
	tempang = CHIMU_DATA.m_attitude.euler.phi;
	*/
	EstimatorSetAtt(CHIMU_DATA.m_attitude.euler.phi, CHIMU_DATA.m_attitude.euler.psi, CHIMU_DATA.m_attitude.euler.theta);
	//EstimatorSetRate(ins_p,ins_q);
	
	//DOWNLINK_SEND_AHRS_EULER(DefaultChannel, &CHIMU_DATA.m_attitude.euler.phi, &CHIMU_DATA.m_attitude.euler.theta, &CHIMU_DATA.m_attitude.euler.psi);

      }
    }
  }
}


//Frequency defined in conf *.xml
void ins_periodic_task( void ) 
{
  // Downlink Send
}

