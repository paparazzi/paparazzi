#include "AMI601.h"



#include "led.h"

#include <string.h>


uint8_t ami601_foo1;
uint8_t ami601_foo2;
uint8_t ami601_foo3;
uint16_t ami601_val[AMI601_NB_CHAN];

volatile uint8_t ami601_status;
volatile bool_t ami601_i2c_done;
volatile uint32_t ami601_nb_err;

/* TRG ( trigger) P0.28 */
#define AMI601_TRG_PIN   28
#define AMI601_TRG_IODIR IO0DIR
#define AMI601_TRG_IOSET IO0SET
#define AMI601_TRG_IOCLR IO0CLR

/* RST ( reset ) P0.29 */
#define AMI601_RST_PIN   29
#define AMI601_RST_IODIR IO0DIR
#define AMI601_RST_IOSET IO0SET
#define AMI601_RST_IOCLR IO0CLR

/* BUSY ( busy ) P0.30 EINT3 */
#define AMI601_BUSY_PIN   
#define AMI601_BUSY_IODIR 
#define AMI601_BUSY_IOSET 
#define AMI601_BUSY_IOCLR 

void ami601_init( void ) {
  /* configure TRG pin as output and assert it*/
  SetBit(AMI601_TRG_IODIR , AMI601_TRG_PIN);
  SetBit(AMI601_TRG_IOSET , AMI601_TRG_PIN);

  /* configure RST pin as output and set it low*/
  SetBit(AMI601_RST_IODIR , AMI601_RST_PIN);
  SetBit(AMI601_RST_IOCLR , AMI601_RST_PIN);


  uint8_t i;
  for (i=0; i< AMI601_NB_CHAN; i++) {
    ami601_val[i] = 0;
  }
  ami601_i2c_done = TRUE;
  ami601_nb_err = 0;
  ami601_status = AMI601_IDLE;

  /* assert reset */
  //  SetBit(AMI601_RST_IOSET , AMI601_RST_PIN);  


}

void ami601_read( void ) {
  if (ami601_status != AMI601_IDLE) {
    ami601_nb_err++;
  }
  else {
    ami601_i2c_done = FALSE;
    ami601_status = AMI601_SENDING_REQ;
    const uint8_t read_cmd[] = { 0x55, 0xAA, 0X14};
    memcpy((void*)i2c_buf, read_cmd, sizeof(read_cmd));
    i2c_transmit(AMI601_SLAVE_ADDR, sizeof(read_cmd), &ami601_i2c_done);
  }
}
