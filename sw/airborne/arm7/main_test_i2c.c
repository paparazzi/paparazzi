
#include "std.h"
#include "sys_time.h"
#include "init_hw.h"
#include "interrupt_hw.h"


#include "messages.h"
#include "downlink.h"
#include "uart.h"

static uint8_t mode;

static inline void main_init( void );
static inline void main_periodic_task( void );
static inline void main_event_task( void);
static inline void main_dl_parse_msg( void );

int main( void ) {
  main_init();
  /* send start_bit */
  I2C0CONSET = 0x60;
  while (1) {
    if (sys_time_periodic())
      main_periodic_task();
    main_event_task();
  }
  return 0;
}

static inline void main_init( void ) {
  hw_init();
  sys_time_init();
  led_init();
  uart0_init_tx();
  int_enable();
}

static inline void main_event_task( void ) {
  if (PprzBuffer()) {
    ReadPprzBuffer();
    if (pprz_msg_received) {
      pprz_parse_payload();
      pprz_msg_received = FALSE;
    }
  }
  if (dl_msg_available) {
    main_dl_parse_msg();
    dl_msg_available = FALSE;
    LED_TOGGLE(2);
  }
}

static inline void main_periodic_task( void ) {
  static uint8_t cnt;
  cnt++;
  if (!(cnt%16)) {
    LED_TOGGLE(1);
    //  DOWNLINK_SEND_MOTOR_BENCH_STATUS(&cpu_time_ticks, &cpu_time_sec, &throttle, &mode);
  }
}

bool_t dl_msg_available;
/** Flag provided to control calls to ::dl_parse_msg. NOT used in this module*/
#define MSG_SIZE 128
uint8_t dl_buffer[MSG_SIZE]  __attribute__ ((aligned));

#include "settings.h"

#define IdOfMsg(x) (x[1])

static inline void main_dl_parse_msg(void) {
  uint8_t msg_id = IdOfMsg(dl_buffer);
  if (msg_id == DL_SETTING) {
    uint8_t i = DL_SETTING_index(dl_buffer);
    float var = DL_SETTING_value(dl_buffer);
    DlSetting(i, var);
    DOWNLINK_SEND_DL_VALUE(&i, &var);
  }  
}


/* SDA0 on P0.3 */
/* SCL0 on P0.2 */

/* A0 A1 A2 are low */
#define SLAVE_ADDR 0x20

void i2c0_ISR(void) __attribute__((naked));

static inline void main_i2c_init ( void ) {
  /* set P0.2 and P0.3 to I2C0 */
  PINSEL0 |= 1 << 4 | 1 << 6;
  /* clear all flags */
  I2C0CONCLR = 0x6C;
  /* enable I2C */
  I2C0CONSET = 0x40;
  /* set bitrate */
  I2C0SCLL = 200;  
  I2C0SCLH = 200;  
  
  //  I2C0CONSET = ;
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_I2C0);  // I2C0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_I2C0);    // I2C0 interrupt enabled
  VICVectCntl9 = VIC_ENABLE | VIC_I2C0;
  VICVectAddr9 = (uint32_t)i2c0_ISR;    // address of the ISR
  
}

void i2c0_ISR(void)
{
  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();
  uint32_t state = I2C0STAT;

  switch (state) {
  case 8:
    /* start condition transmitted */
    /* send slave addr + W */
    I2C0DAT = 0x74;
    /* clear SI and start flag */
    I2C0CONCLR = 0x28;
    
    break;
    
  case 24:
    /* ack received from slave for address */
    /* send data */
    I2C0DAT = 0x55;
    /* clear SI */
    I2C0CONCLR = 0x8;
  break;
  
  /* ack received from slave for byte */
  case 40:
    /* transmitt stop condition */
    I2C0CONSET = 0x10;
    /* clear SI */
    I2C0CONCLR = 0x8;
    break;

  default:
    break;
  }

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}
