#include "pt_ant_sensors.h"

#include "LPC21xx.h"
#include "armVIC.h"

void SPI0_ISR(void) __attribute__((naked));
#define S0SPCR_SPIE  7  /* SPI enable */
#define PT_ANT_SENSOR_SS_PIN 3
#define PtAntSensorsSelect() { SetBit(IO0SET, PT_ANT_SENSOR_SS_PIN);}
#define PtAntSensorsUnselect() { SetBit(IO0CLR, PT_ANT_SENSOR_SS_PIN);}

volatile uint8_t pt_ant_sensors_data_available;
struct PtAntSensorData pt_ant_sensors_data;

volatile uint8_t pt_ant_sensors_buffer_idx;
uint8_t* pt_ant_sensors_buffer = (uint8_t*)&pt_ant_sensors_data;

void pt_ant_sensors_init(void) {
  pt_ant_sensors_data_available = FALSE;

}


void pt_ant_sensors_read(void) {
  PtAntSensorsSelect();
  pt_ant_sensors_buffer_idx = 0;  
  S0SPDR = 0xAA;
}




void pt_ant_sensors_init_spi(void) {
  /* init SPI0 */
  /* setup pins function for sck, miso, mosi */
  PINSEL0 |= 1<<8 | 1<<10| 1<<12;
  /* setup ssel as output */
  IO0DIR |= 1<<PT_ANT_SENSOR_SS_PIN;
  /* unselect sensor */
  PtAntSensorsUnselect();
  /* configure SPI : 8 bits CPOL=0 CPHA=0 MSB_first master */
  S0SPCR = 1<<5;
  /* setup SPI clock rate : PCLK / 32 */
  S0SPCCR = 0x20;

  /* initialize interrupt vector */
  VICIntSelect &= ~VIC_BIT(VIC_SPI0);   // SPI0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_SPI0);     // SPI0 interrupt enabled
  VICVectCntl10 = VIC_ENABLE | VIC_SPI0;
  VICVectAddr10 = (uint32_t)SPI0_ISR;    // address of the ISR

  /* clear pending interrupt */
  SetBit(S0SPINT, SPI0IF);
  /* enable SPI interrupt */
  SetBit(S0SPCR, S0SPCR_SPIE);
}


void SPI0_ISR(void) {
  ISR_ENTRY();
  /* FIXME : do something usefull with the status register reading */	\
  uint8_t foo __attribute__((unused)) = S0SPSR;				\
  pt_ant_sensors_buffer[pt_ant_sensors_buffer_idx] = S0SPDR;
  pt_ant_sensors_buffer_idx++;			
  if (pt_ant_sensors_buffer_idx < sizeof(pt_ant_sensors_data)) {
    S0SPDR = 0xAA;
  }					
  else {
    PtAntSensorsUnselect();
    pt_ant_sensors_data_available = TRUE;
  }	
  
  /* clear the interrupt */
  S0SPINT = _BV(SPI0IF); 
  /* clear this interrupt from the VIC */
  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
