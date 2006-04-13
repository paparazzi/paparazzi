#include "servos_4015_hw.h"
#include "command.h"
#include "armVIC.h"

#include "airframe.h"
#include "sys_time.h"
//#include "led.h"
const pprz_t failsafe_values[COMMANDS_NB] = COMMANDS_FAILSAFE;

#define COMMAND(i) servos_values[i]
#define SERVOS_TICS_OF_USEC(s) SYS_TICS_OF_USEC(s)



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




void PWM_ISR ( void ) {
  ISR_ENTRY();
  //  LED_TOGGLE(2);
  if (servos_idx == 0) {
    IO1CLR = _BV(SERV0_RESET_PIN);
    IO1SET = _BV(SERV0_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
    PWMLER = PWMLER_LATCH0;
    servos_idx++;
  }
  else if (servos_idx < NB_SERVOS) {
    IO1CLR = _BV(SERV0_DATA_PIN);
    PWMMR0 = servos_values[servos_idx];
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
