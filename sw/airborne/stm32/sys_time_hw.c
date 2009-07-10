#include "sys_time.h"


volatile bool_t sys_time_period_elapsed;
uint32_t cpu_time_ticks;

void sys_time_init( void ) {
  /* Generate SysTick interrupt every PERIODIC_TASK_PERIOD AHS clk */
  if(SysTick_Config(PERIODIC_TASK_PERIOD-1)) {
    while(1);
  }
  /* Set SysTick handler */
  NVIC_SetPriority(SysTick_IRQn, 0x0);
  sys_time_period_elapsed = FALSE;
  cpu_time_ticks = 0;
}

void sys_tick_irq_handler(void) {
  sys_time_period_elapsed = TRUE;
}


