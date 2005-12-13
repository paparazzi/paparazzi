#include "servos.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include "types.h"
#include "config.h"


#define CLOCK_OF_US(us) ((us)*(PCLK/1000000))

#define SERV4_START_POS 1000
#define SERV5_START_POS 1200
#define SERV6_START_POS 1400
#define SERV7_START_POS 1600


#define NB_SERVOS 4
#define SERVO_REFRESH_US 25000
uint32_t servos_values[NB_SERVOS] = 
  { CLOCK_OF_US(SERV4_START_POS), 
    CLOCK_OF_US(SERV5_START_POS), 
    CLOCK_OF_US(SERV6_START_POS), 
    CLOCK_OF_US(SERV7_START_POS)
  };
uint32_t servos_delay = CLOCK_OF_US(SERVO_REFRESH_US - SERV4_START_POS - SERV5_START_POS - SERV6_START_POS - SERV7_START_POS) / 2; 
uint8_t servos_idx = 0;


/* 
   servo0
   clock : PWM5 ( P0.21 )
   data  :        P1.20
   reset :        p1.21

   servo 1 
   clock : PWM2 ( P0.7 )
   data  :        P1.30
   reset :        P1.29
*/

void servos_init ( void ) {
  /* PWM selected as IRQ */
  VICIntSelect &= ~VIC_BIT(VIC_PWM);   
  /* PWM interrupt enabled */
  VICIntEnable = VIC_BIT(VIC_PWM);  
  VICVectCntl3 = VIC_ENABLE | VIC_PWM;
  /* address of the ISR */
  VICVectAddr3 = (uint32_t)PWM_ISR;
  /* PW2 pin (P0.7) used for PWM  */
  PINSEL0 |= 0x02 << 14;
  /* set match2 to go of a long time from now */
  PWMMR0 = 0XFFFFFF;  
  PWMMR2 = 0XFFF;  
  /* commit above change        */
  PWMLER = 0x5;
  /* interrupt on PWMMR2 match  */
  PWMMCR = 1 << 6;
  /* enable PWM2 ouptput        */
  PWMPCR = 1 << 10;
  /* enable PWM timer counter and PWM mode  */
  PWMTCR = 0x9; 
}

void PWM_ISR ( void ) {
  ISR_ENTRY();
  if (servos_idx == 0) {
    IO1CLR = SERV1_RESET_BIT;
    IO1SET = SERV1_DATA_BIT;
    PWMMR0 = servos_values[servos_idx];
    PWMLER = 0x1;
    servos_idx++;
  }
  else if (servos_idx < NB_SERVOS) {
    IO1CLR = SERV1_DATA_BIT;
    PWMMR0 = servos_values[servos_idx];
    PWMLER = 0x1;
    servos_idx++;
  }
  else {
    IO1SET = SERV1_RESET_BIT;
    PWMMR0 = servos_delay;
    PWMLER = 0x1;
      servos_idx = 0;
  }
  /* clear the interrupt */
  PWMIR = 1<<2;
  VICVectAddr = 0x00000000;
  ISR_EXIT();  
}

