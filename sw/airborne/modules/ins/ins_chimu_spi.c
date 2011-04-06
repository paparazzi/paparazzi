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

#define SPI_INS_BUF_SIZE	128

uint8_t spi_ins_in[SPI_INS_BUF_SIZE];
uint8_t spi_ins_out[SPI_INS_BUF_SIZE];


CHIMU_PARSER_DATA CHIMU_DATA;

INS_FORMAT ins_roll_neutral;
INS_FORMAT ins_pitch_neutral;

void ins_init( void ) 
{
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  
  CHIMU_Init(&CHIMU_DATA);  
  
  for (int i=0;i<SPI_INS_BUF_SIZE;i++)
    spi_ins_out[i] = 0x00;
  
  spi_buffer_input = spi_ins_in;
  spi_buffer_output = spi_ins_out;
  spi_buffer_length = 1;

  spi_message_received = FALSE;

  SpiEnable();

/*                              \
  SpiInitBuf();                             \
  SpiEnableTxi();          \
*/
  
}

void parse_ins_msg( void )
{
  if (spi_message_received) 
  {
//    LED_TOGGLE(3);

    /* Got a message on SPI. */
    spi_message_received = FALSE;
    //chimu_spi_parse(spi_buffer_input);
  }
  
/*  if (InsBuffer()) 
  {
    while (InsLink(ChAvailable()))
    {
      uint8_t ch = InsLink(Getch());
      
      if (CHIMU_Parse(ch, 0, &CHIMU_DATA))
      {
        if(CHIMU_DATA.m_MsgID==0x03)
        {
	  //RunOnceEvery(25, LED_TOGGLE(3) );
	  if (CHIMU_DATA.m_attitude.euler.phi > M_PI)
	  {
	    CHIMU_DATA.m_attitude.euler.phi -= 2 * M_PI;
	  }
          EstimatorSetAtt(CHIMU_DATA.m_attitude.euler.phi, CHIMU_DATA.m_attitude.euler.psi, CHIMU_DATA.m_attitude.euler.theta);
          //EstimatorSetRate(ins_p,ins_q);
        }
      }
    }
  }
*/
  
}


//Frequency defined in conf *.xml
void ins_periodic_task( void ) 
{
  // Downlink Send
}

