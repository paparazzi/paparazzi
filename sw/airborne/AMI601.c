#include "AMI601.h"

#include "i2c.h"

#include "led.h"

#include <string.h>

#define FOO_DELAY() {  \
    uint16_t foo = 1;  \
    while (foo) foo++; \
  }


#define AMI601_SLAVE_ADDR 0x60

float ami601_ax;
float ami601_ay;
float ami601_az;

float ami601_mx;
float ami601_my;
float ami601_mz;

uint8_t ami601_foo1;
uint8_t ami601_foo2;
uint8_t ami601_foo3;

uint16_t ami601_val[AMI601_NB_CHAN];
static volatile bool_t ami601_i2c_done;

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


  FOO_DELAY();
  /* assert reset */
  SetBit(AMI601_RST_IOSET , AMI601_RST_PIN);  


}

void ami601_periodic( void ) {
  /* pulse TRG */
  //  SetBit(AMI601_TRG_IOCLR , AMI601_TRG_PIN);
  //  FOO_DELAY();
  //  SetBit(AMI601_TRG_IOSET , AMI601_TRG_PIN);

  if (ami601_i2c_done) {
    LED_ON(2);
    const uint8_t read_cmd[] = { 0x55, 0xAA, 0X14};
    memcpy((void*)i2c_buf, read_cmd, sizeof(read_cmd));
    i2c_transmit(AMI601_SLAVE_ADDR, sizeof(read_cmd), &ami601_i2c_done);
    ami601_i2c_done = FALSE;

    while (!ami601_i2c_done);

    FOO_DELAY();

    i2c_receive(AMI601_SLAVE_ADDR, 15, &ami601_i2c_done);
    while (!ami601_i2c_done);
    LED_OFF(2);
    
    ami601_foo1 = i2c_buf[0];
    ami601_foo1 = i2c_buf[1];
    ami601_foo1 = i2c_buf[2];

    uint8_t i;
    for (i=0; i< AMI601_NB_CHAN; i++) {
      ami601_val[i] = 	i2c_buf[3 + 2 * i];
      ami601_val[i] +=  i2c_buf[3 + 2 * i + 1]  * 256;
    }

    

  }

}



void ami601_send_sleep( void ) {


}

#define AX_NEUTRAL 2323
#define AY_NEUTRAL 2323
#define AZ_NEUTRAL 2323

#define AX_GAIN 0.0283936
#define AY_GAIN -0.0283936
#define AZ_GAIN 0.0283936

#define MX_NEUTRAL 2048
#define MY_NEUTRAL 2048
#define MZ_NEUTRAL 2048

#define MX_GAIN -1.
#define MY_GAIN  1.
#define MZ_GAIN  1.

void ami601_scale_measures(void) {

  ami601_ax = (ami601_val[3] - AX_NEUTRAL) * AX_GAIN;
  ami601_ay = (ami601_val[5] - AY_NEUTRAL) * AY_GAIN;
  ami601_az = (ami601_val[1] - AZ_NEUTRAL) * AZ_GAIN;

  ami601_mx = (ami601_val[0] - MX_NEUTRAL) * MX_GAIN;
  ami601_my = (ami601_val[4] - MY_NEUTRAL) * MY_GAIN;
  ami601_mz = (ami601_val[2] - MZ_NEUTRAL) * MZ_GAIN;

}
