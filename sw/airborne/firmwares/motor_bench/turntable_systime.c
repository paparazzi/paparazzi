#include "armVIC.h"
#include "mcu_periph/sys_time.h"
#include "led.h"

#define SYS_TICK_IT TIR_MR0I

#define TIMER0_IT_MASK (SYS_TICK_IT  |  \
                        TIR_CR0I)

void sys_time_arch_init(void)
{
  /* setup Timer 0 to count forever  */
  /* reset & disable timer 0         */
  T0TCR = TCR_RESET;
  /* set the prescale divider        */
  T0PR = T0_PCLK_DIV - 1;
  /* enable interrupt on match0      */
  T0MCR = TMCR_MR0_I;
  /* disable capture registers       */
  T0CCR = 0;
  /* disable external match register */
  T0EMR = 0;
  /* enable timer 0                  */
  T0TCR = TCR_ENABLE;

  /* set first sys tick interrupt    */
  T0MR0 = sys_time.resolution_cpu_ticks;

  /* select TIMER0 as IRQ    */
  VICIntSelect &= ~VIC_BIT(VIC_TIMER0);
  /* enable TIMER0 interrupt */
  VICIntEnable = VIC_BIT(VIC_TIMER0);
  /* on slot vic slot 1      */
  _VIC_CNTL(TIMER0_VIC_SLOT) = VIC_ENABLE | VIC_TIMER0;
  /* address of the ISR      */
  _VIC_ADDR(TIMER0_VIC_SLOT) = (uint32_t)TIMER0_ISR;
}

static inline void sys_tick_irq_handler(void)
{
  /* set match register for next interrupt */
  T0MR0 += ticks_resolution - 1;

  sys_time.nb_tick++;
  sys_time.nb_sec_rem += sys_time.resolution_cpu_ticks;
  if (sys_time.nb_sec_rem >= sys_time.ticks_per_sec) {
    sys_time.nb_sec_rem -= sys_time.ticks_per_sec;
    sys_time.nb_sec++;
#ifdef SYS_TIME_LED
    LED_TOGGLE(SYS_TIME_LED);
#endif
  }
  for (unsigned int i = 0; i < SYS_TIME_NB_TIMER; i++) {
    if (sys_time.timer[i].in_use &&
        sys_time.nb_tick >= sys_time.timer[i].end_time) {
      sys_time.timer[i].end_time += sys_time.timer[i].duration;
      sys_time.timer[i].elapsed = TRUE;
      if (sys_time.timer[i].cb) { sys_time.timer[i].cb(i); }
    }
  }
}

extern uint32_t lp_pulse;
extern uint32_t nb_pulse;

void TIMER0_ISR(void)
{
  ISR_ENTRY();

  while (T0IR & TIMER0_IT_MASK) {

    if (T0IR & SYS_TICK_IT) {
      sys_tick_irq_handler();
      T0IR = SYS_TICK_IT;
    }

    if (T0IR & TIR_CR0I) {
      static uint32_t pulse_last_t;
      uint32_t t_now = T0CR0;
      uint32_t diff = t_now - pulse_last_t;
      lp_pulse = (lp_pulse + diff) / 2;
      pulse_last_t = t_now;
      nb_pulse++;
      //    got_one_pulse = TRUE;
      T0IR = TIR_CR0I;
    }
  }

  VICVectAddr = 0x00000000;
  ISR_EXIT();
}
