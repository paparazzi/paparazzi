#include "actuators/booz_actuators_mkk.h"

#include "i2c.h"

#include <stm32/rcc.h>
#include <stm32/misc.h>

void tim2_irq_handler(void);

/* ---------------------------------------------------------------
    TIM2 Configuration: Output Compare Timing Mode:
    TIM2CLK = 36 MHz, Prescaler = 4, TIM2 counter clock = 7.2 MHz
*/

void booz_actuators_mkk_arch_init(void) {			

  /* TIM2 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /* Time base configuration */			
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_TimeBaseStructure.TIM_Period = 3072;		
  //  TIM_TimeBaseStructure.TIM_Period = 2048;		
  TIM_TimeBaseStructure.TIM_Prescaler = 0;		
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;	
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
  /* Prescaler configuration */
  TIM_PrescalerConfig(TIM2, 4, TIM_PSCReloadMode_Immediate);

  /* Enable the TIM2 global Interrupt */		
  NVIC_InitTypeDef NVIC_InitStructure;		
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  NVIC_Init(&NVIC_InitStructure);			
  
  //  DEBUG5_INIT();

  TIM_Cmd(TIM2, ENABLE);										

}

void tim2_irq_handler(void) {
  //  DEBUG5_T();

  /* Clear TIM2 update interrupt */
  TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
  actuators_mkk.idx++;							
  if (actuators_mkk.idx<ACTUATORS_MKK_NB) {				
    DeviceBuf[0] = supervision.commands[actuators_mkk.idx];		
    DeviceTransmit(actuators_addr[actuators_mkk.idx], 1, &actuators_mkk.i2c_done);
  }								
  else {
    actuators_mkk.status = IDLE;
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
  }
}
