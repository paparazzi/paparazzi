#include "actuators.h"
#include "armVIC.h"

#include "airframe.h"
#include "sys_time.h"


void actuators_init ( void ) {
  /* PWM selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_PWM);   
  /* PWM interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_PWM);  
  VICVectCntl3 = VIC_ENABLE | VIC_PWM;
  /* address of the ISR */
  VICVectAddr3 = (uint32_t)PWM_ISR;
  /* PW5 pin (P0.21) used for PWM  */
  IO0DIR |= _BV(SERV0_CLOCK_PIN);
  IO1DIR |= _BV(SERV0_DATA_PIN) | _BV(SERV0_RESET_PIN);
  SERV0_CLOCK_PINSEL |= SERV0_CLOCK_PINSEL_VAL << SERV0_CLOCK_PINSEL_BIT;

  /* set match5 to go of a long time from now */
  PWMMR0 = 0XFFFFFF;  
  //PWMMR0 = CLOCK_OF_US(1500);  
  PWMMR5 = 0XFFF;  
  /* commit above change        */
  PWMLER = PWMLER_LATCH0 | PWMLER_LATCH5;
  /* interrupt on PWMMR5 match  */
  PWMMCR = PWMMCR_MR0R | PWMMCR_MR5I;
  /* enable PWM5 ouptput        */
  PWMPCR = PWMPCR_ENA5;

  /* Prescaler */
  PWMPR = PWM_PRESCALER-1;

  /* enable PWM timer counter and PWM mode  */
  PWMTCR = PWMTCR_COUNTER_ENABLE | PWMTCR_PWM_ENABLE; 
  /* Load failsafe values              */
   /* Set all servos at their midpoints */
  /* compulsory for unaffected servos  */
  uint8_t i;
  for( i=0 ; i < _4015_NB_CHANNELS ; i++ )
    servos_values[i] = SERVOS_TICS_OF_USEC(1500);
}

uint16_t servos_values[_4015_NB_CHANNELS];

#define CLOCK_OF_US(us) ((us)*(PCLK/1000000))

#define SERV4_START_POS 1000
#define SERV5_START_POS 1200
#define SERV6_START_POS 1400
#define SERV7_START_POS 1600


#define SERVO_REFRESH_TICS SERVOS_TICS_OF_USEC(25000)


static uint8_t servos_idx = 0;
static uint32_t servos_delay;

void PWM_ISR ( void ) {
  ISR_ENTRY();
  //  LED_TOGGLE(2);
  if (servos_idx == 0) {
    IO1CLR = _BV(SERV0_RESET_PIN);
    IO1SET = _BV(SERV0_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
    servos_delay = SERVO_REFRESH_TICS - servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0;
    servos_idx++;
  }
  else if (servos_idx < _4015_NB_CHANNELS) {
    IO1CLR = _BV(SERV0_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
    servos_delay -= servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0;
    servos_idx++;
  }
  else {
    IO1SET = _BV(SERV0_RESET_PIN);
    PWMMR0 = servos_delay;
    PWMLER = PWMLER_LATCH0;
    servos_idx = 0;
  }
  /* clear the interrupt */
  PWMIR = PWMIR_MR5I;
  VICVectAddr = 0x00000000;
  ISR_EXIT();  
}
