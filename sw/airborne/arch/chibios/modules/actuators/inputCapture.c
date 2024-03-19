#include "inputCapture.h"


enum  TimICChannel {TIMIC_CH1=1<<0, TIMIC_CH2=1<<1, TIMIC_CH3=1<<2, TIMIC_CH4=1<<3};

static const TimICDriver*  driverByTimerIndex[6] = {NULL};


static void input_capture_lld_serve_interrupt(const TimICDriver * const timicp) __attribute__((unused));
static void _input_capture_isr_invoke_capture_cb(const TimICDriver * const timicp, uint32_t channel);
static void _input_capture_isr_invoke_overflow_cb(const TimICDriver * const timicp);

void timIcObjectInit(TimICDriver *timicp)
{
  timicp->config = NULL;
  timicp->state = TIMIC_STOP;
  timicp->dier = 0;
}

void timIcStart(TimICDriver *timicp, const TimICConfig *configp)
{
  osalDbgCheck((configp != NULL) && (timicp != NULL));
  osalDbgAssert((configp->prescaler >= 1) &&
		(configp->prescaler <= 65536),
		"prescaler must be 1 .. 65536");
  osalDbgAssert(timicp->state == TIMIC_STOP, "state error");
  timicp->config = configp;
  stm32_tim_t * const timer = timicp->config->timer;
  chMtxObjectInit(&timicp->mut);
  timIcRccEnable(timicp);
  timicp->channel = 0;
  if (timicp->config->active & (CH1_RISING_EDGE | CH1_FALLING_EDGE | CH1_BOTH_EDGES))
    timicp->channel |= TIMIC_CH1;
  if (timicp->config->active & (CH2_RISING_EDGE | CH2_FALLING_EDGE | CH2_BOTH_EDGES))
    timicp->channel |= TIMIC_CH2;
  if (timicp->config->active & (CH3_RISING_EDGE | CH3_FALLING_EDGE | CH3_BOTH_EDGES))
    timicp->channel |= TIMIC_CH3;
  if (timicp->config->active & (CH4_RISING_EDGE | CH4_FALLING_EDGE | CH4_BOTH_EDGES))
    timicp->channel |= TIMIC_CH4;
  
  timer->CR1 = 0;	    // disable timer

  // hack in case of timer with more fields
#if defined (STM32G0XX)  || defined (STM32G4XX)|| defined (STM32H7XX)

  TIM_TypeDef *cmsisTimer = (TIM_TypeDef *) timer;
  cmsisTimer->CCMR3 = cmsisTimer->AF1 = cmsisTimer->AF2 =
    cmsisTimer->TISEL = 0;
#endif
  
  timer->PSC = configp->prescaler - 1U;	 // prescaler
  timer->ARR = configp->arr ? configp->arr : 0xffffffff;
  timer->DCR = configp->dcr;
  if (timicp->config->mode == TIMIC_PWM_IN) {
  chDbgAssert(__builtin_popcount(timicp->channel) == 1, "In pwm mode, only one channel must be set");
  chDbgAssert((timicp->config->capture_cb == NULL) && (timicp->config->overflow_cb == NULL),
	      "In pwm mode, callback are not implemented, use PWMDriver instead");
  switch (timicp->channel) {
  case TIMIC_CH1:
    timer->CCMR1 = (0b01 << TIM_CCMR1_CC1S_Pos) | (0b10 << TIM_CCMR1_CC2S_Pos);
    timer->CCMR2 = 0U;
    timer->CCER = TIM_CCER_CC2P; /* CC1P et CC1NP = 0 */
    timer->SMCR = (0b101 << TIM_SMCR_TS_Pos) | (0b100 << TIM_SMCR_SMS_Pos);
    timer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);         
    break;
  case TIMIC_CH2:
    timer->CCMR1 = (0b10 << TIM_CCMR1_CC1S_Pos) | (0b01 << TIM_CCMR1_CC2S_Pos);
    timer->CCMR2 = 0U;
    timer->CCER = TIM_CCER_CC1P;   /* CC2P et CC2NP = 0 */      
    timer->SMCR = (0b110 << TIM_SMCR_TS_Pos) | (0b100 << TIM_SMCR_SMS_Pos);
    timer->CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E); 
    break;
  case TIMIC_CH3:
    timer->CCMR2 = (0b01 << TIM_CCMR2_CC3S_Pos) | (0b10 << TIM_CCMR2_CC4S_Pos);
    timer->CCMR1 = 0U;
    timer->CCER =  TIM_CCER_CC4P;    /* CC3P et CC3NP = 0 */ 
    timer->SMCR = (0b101 << TIM_SMCR_TS_Pos) | (0b100 << TIM_SMCR_SMS_Pos);
    timer->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E); 
    break;
  case TIMIC_CH4:
    timer->CCMR2 = (0b10 << TIM_CCMR2_CC3S_Pos) | (0b01 << TIM_CCMR2_CC4S_Pos);
    timer->CCMR1 = 0U;
    timer->CCER = TIM_CCER_CC3P; /* CC4P et CC4NP = 0 */ 
    timer->SMCR = (0b110 << TIM_SMCR_TS_Pos) | (0b100 << TIM_SMCR_SMS_Pos);
    timer->CCER |= (TIM_CCER_CC3E | TIM_CCER_CC4E);
    break;
  default:
    chSysHalt("channel must be TIMIC_CH1 .. TIMIC_CH4");
  }
  } else if (timicp->config->mode == TIMIC_INPUT_CAPTURE) {
    /*
      Select the active input: TIMx_CCR1 must be linked to the TI1 input, so write the CC1S
      bits to 01 in the TIMx_CCMR1 register. As soon as CC1S becomes different from 00,
      the channel is configured in input and the TIMx_CCR1 register becomes read-only.
      •
      Select the edge of the active transition on the TI1 channel by writing CC1P and CC1NP
      bits to 0 in the TIMx_CCER register (rising edge in this case).
      •
      •
      Enable capture from the counter into the capture register by setting the CC1E bit in the
      TIMx_CCER register.
      •
      If needed, enable the related interrupt request by setting the CC1IE bit in the
      TIMx_DIER register, and/or the DMA request by setting the CC1DE bit in the
      TIMx_DIER register.
     */


    timer->CCMR1 = 0;
    timer->CCMR2 = 0;     
    timer->CCER  = 0; 
    timer->SMCR  = 0; 
    timer->CCER = 0;
    
    if (timicp->channel & TIMIC_CH1) {
      switch (timicp->config->active & (CH1_RISING_EDGE | CH1_FALLING_EDGE | CH1_BOTH_EDGES)) {
      case CH1_RISING_EDGE:
	timer->CCER |= 0;
	break;
      case CH1_FALLING_EDGE:
	timer->CCER |= TIM_CCER_CC1P;
	break;
      case CH1_BOTH_EDGES: 
	timer->CCER |= (TIM_CCER_CC1P | TIM_CCER_CC1NP);
	break;
      default:
      chSysHalt("No configuration given for CH1");
      }
      timer->CCMR1 |= (0b01 << TIM_CCMR1_CC1S_Pos); 
      timer->CCER |= TIM_CCER_CC1E;
      timicp->dier |= STM32_TIM_DIER_CC1IE;
    }
    if (timicp->channel & TIMIC_CH2) {
      switch (timicp->config->active & (CH2_RISING_EDGE | CH2_FALLING_EDGE | CH2_BOTH_EDGES)) {
      case CH2_RISING_EDGE:
	timer->CCER |= 0;
	break;
      case CH2_FALLING_EDGE:
	timer->CCER |= TIM_CCER_CC2P;
	break;
      case CH2_BOTH_EDGES: 
	timer->CCER |= (TIM_CCER_CC2P | TIM_CCER_CC2NP);
	break;
       default:
	chSysHalt("No configuration given for CH2");
      }
      timer->CCMR1 |= (0b01 << TIM_CCMR1_CC2S_Pos);
      timer->CCER |= TIM_CCER_CC2E;         
      timicp->dier |= STM32_TIM_DIER_CC2IE;
    }
    if (timicp->channel & TIMIC_CH3) {
      switch (timicp->config->active & (CH3_RISING_EDGE | CH3_FALLING_EDGE | CH3_BOTH_EDGES)) {
      case CH3_RISING_EDGE:
	timer->CCER |= 0;
	break;
      case CH3_FALLING_EDGE:
	timer->CCER |= TIM_CCER_CC3P;
	break;
      case CH3_BOTH_EDGES: 
	timer->CCER |= (TIM_CCER_CC3P | TIM_CCER_CC3NP);
	break;
       default:
	chSysHalt("No configuration given for CH3");
      }
      timer->CCMR2 |= (0b01 << TIM_CCMR2_CC3S_Pos);
      timer->CCER |= TIM_CCER_CC3E;         
      timicp->dier |= STM32_TIM_DIER_CC3IE;
    }
    if (timicp->channel & TIMIC_CH4) {
      switch (timicp->config->active & (CH4_RISING_EDGE | CH4_FALLING_EDGE | CH4_BOTH_EDGES)) {
      case CH4_RISING_EDGE:
	timer->CCER |= 0;
	break;
      case CH4_FALLING_EDGE:
	timer->CCER |= TIM_CCER_CC4P;
	break;
      case CH4_BOTH_EDGES: 
	timer->CCER |= (TIM_CCER_CC4P | TIM_CCER_CC4NP);
	break;
    default:
	chSysHalt("No configuration given for CH4");
      }
      timer->CCMR2 |= (0b01 << TIM_CCMR2_CC4S_Pos);
      timer->CCER |= TIM_CCER_CC4E;
      timicp->dier |= STM32_TIM_DIER_CC4IE;
    }
  } else { 
    chSysHalt("invalid mode");
  }
  // keep only DMA bits, not ISR bits that are handle by driver
  if (timicp->config->capture_cb == NULL)
    timicp->dier = 0;
  if (timicp->config->overflow_cb)
    timicp->dier |= STM32_TIM_DIER_UIE;
  
  timicp->dier = timicp->dier | (timicp->config->dier & (~ STM32_TIM_DIER_IRQ_MASK)); 
  timicp->state = TIMIC_READY;
}

void timIcStartCapture(TimICDriver *timicp)
{
  osalDbgCheck(timicp != NULL);
  osalDbgAssert(timicp->state == TIMIC_READY, "state error");
  stm32_tim_t * const timer = timicp->config->timer;
  osalDbgCheck(timer != NULL);
  timer->CR1 = STM32_TIM_CR1_URS;
  timer->EGR |= STM32_TIM_EGR_UG;
  timer->SR = 0;
  timer->DIER = timicp->dier;
  timer->CR1 = STM32_TIM_CR1_URS | STM32_TIM_CR1_CEN;
  timicp->state = TIMIC_ACTIVE;
}

void timIcStopCapture(TimICDriver *timicp)
{
  osalDbgCheck(timicp != NULL);
  osalDbgAssert(timicp->state != TIMIC_STOP, "state error");
  stm32_tim_t * const timer = timicp->config->timer;
  osalDbgCheck(timer != NULL);
  timer->CR1 &= ~TIM_CR1_CEN;
  timer->DIER = 0;
  timicp->state = TIMIC_READY;
}

void timIcStop(TimICDriver *timicp)
{
  osalDbgAssert(timicp->state != TIMIC_STOP, "state error");
  chMtxLock(&timicp->mut);
  timIcRccDisable(timicp);
  timIcObjectInit(timicp);
  chMtxUnlock(&timicp->mut);
  timicp->state = TIMIC_STOP;
  timicp->dier = 0;
}




void timIcRccEnable(const TimICDriver * const timicp)
{
  const stm32_tim_t * const timer = timicp->config->timer;
  const bool use_isr = timicp->config->capture_cb || timicp->config->overflow_cb;
#ifdef TIM1
  if (timer == STM32_TIM1) {
    driverByTimerIndex[0] = timicp;
    rccEnableTIM1(true);
    rccResetTIM1();
    if (use_isr) {
#ifdef STM32_TIM1_UP_TIM10_NUMBER 
      nvicEnableVector(STM32_TIM1_UP_TIM10_NUMBER, STM32_IRQ_TIM1_UP_TIM10_PRIORITY);
#endif
#ifdef STM32_TIM1_CC_NUMBER 
      nvicEnableVector(STM32_TIM1_CC_NUMBER, STM32_IRQ_TIM1_CC_PRIORITY);
#endif
    }
  }
#endif
#ifdef TIM2
  else  if (timer == STM32_TIM2) {
    driverByTimerIndex[1] = timicp;
    rccEnableTIM2(true);
    rccResetTIM2();
    if (use_isr) {
      nvicEnableVector(STM32_TIM2_NUMBER, STM32_IRQ_TIM2_PRIORITY);
    }
  }
#endif
#ifdef TIM3
  else  if (timer == STM32_TIM3) {
    driverByTimerIndex[2] = timicp;
    rccEnableTIM3(true);
    rccResetTIM3();
    if (use_isr) {
      nvicEnableVector(STM32_TIM3_NUMBER, STM32_IRQ_TIM3_PRIORITY);
    }
  }
#endif
#ifdef TIM4
  else  if (timer == STM32_TIM4) {
    driverByTimerIndex[3] = timicp;
    rccEnableTIM4(true);
    rccResetTIM4();
    if (use_isr) {
      nvicEnableVector(STM32_TIM4_NUMBER, STM32_IRQ_TIM4_PRIORITY);
    }
  }
#endif
#ifdef TIM5
  else  if (timer == STM32_TIM5) {
    driverByTimerIndex[4] = timicp;
    rccEnableTIM5(true);
    rccResetTIM5();
    if (use_isr) {
      nvicEnableVector(STM32_TIM5_NUMBER, STM32_IRQ_TIM5_PRIORITY);
    }
  }
#endif
#ifdef TIM8
  else  if (timer == STM32_TIM8) {
    driverByTimerIndex[5] = timicp;
    rccEnableTIM8(true);
    rccResetTIM8();
    if (use_isr) {
#ifdef STM32_TIM8_UP_TIM13_NUMBER 
      nvicEnableVector(STM32_TIM8_UP_TIM13_NUMBER, STM32_IRQ_TIM8_UP_TIM13_PRIORITY);
#endif
#ifdef STM32_TIM8_CC_NUMBER 
      nvicEnableVector(STM32_TIM8_CC_NUMBER, STM32_IRQ_TIM8_CC_PRIORITY);
#endif
    }
  }
#endif
#ifdef TIM9
  else  if (timer == STM32_TIM9) {
    rccEnableTIM9(true);
    rccResetTIM9();
  }
#endif
#ifdef TIM10
  else  if (timer == STM32_TIM10) {
    rccEnableTIM10(true);
    rccResetTIM10();
  }
#endif
#ifdef TIM11
  else  if (timer == STM32_TIM11) {
    rccEnableTIM11(true);
    rccResetTIM11();
  }
#endif
#ifdef TIM12
  else  if (timer == STM32_TIM12) {
    rccEnableTIM12(true);
    rccResetTIM12();
  }
#endif
#ifdef TIM13
  else  if (timer == STM32_TIM13) {
    rccEnableTIM13(true);
    rccResetTIM13();
  }
#endif
#ifdef TIM14
  else  if (timer == STM32_TIM14) {
    rccEnableTIM14(true);
    rccResetTIM14();
  }
#endif
#ifdef TIM15
  else  if (timer == STM32_TIM15) {
    rccEnableTIM15(true);
    rccResetTIM15();
  }
#endif
#ifdef TIM16
  else  if (timer == STM32_TIM16) {
    rccEnableTIM16(true);
    rccResetTIM16();
  }
#endif
#ifdef TIM17
  else  if (timer == STM32_TIM17) {
    rccEnableTIM17(true);
    rccResetTIM17();
  }
#endif
#ifdef TIM18
  else  if (timer == STM32_TIM18) {
    rccEnableTIM18(true);
    rccResetTIM18();
  }
#endif
#ifdef TIM19
  else  if (timer == STM32_TIM19) {
    rccEnableTIM19(true);
    rccResetTIM19();
  }
#endif
  else {
    chSysHalt("not a valid timer");
  }
};

void timIcRccDisable(const TimICDriver * const timicp)
{
  const stm32_tim_t * const timer = timicp->config->timer;
#ifdef TIM1
  if (timer == STM32_TIM1) {
    rccResetTIM1();
    rccDisableTIM1();
  }
#endif
#ifdef TIM2
  else  if (timer == STM32_TIM2) {
    rccResetTIM2();
    rccDisableTIM2();
  }
#endif
#ifdef TIM3
  else  if (timer == STM32_TIM3) {
    rccResetTIM3();
    rccDisableTIM3();
  }
#endif
#ifdef TIM4
  else  if (timer == STM32_TIM4) {
    rccResetTIM4();
    rccDisableTIM4();
  }
#endif
#ifdef TIM5
  else  if (timer == STM32_TIM5) {
    rccResetTIM5();
    rccDisableTIM5();
  }
#endif
#ifdef TIM8
  else  if (timer == STM32_TIM8) {
    rccResetTIM8();
    rccDisableTIM8();
  }
#endif
#ifdef TIM9
  else  if (timer == STM32_TIM9) {
    rccResetTIM9();
    rccDisableTIM9();
  }
#endif
#ifdef TIM10
  else  if (timer == STM32_TIM10) {
    rccResetTIM10();
    rccDisableTIM10();
  }
#endif
#ifdef TIM11
  else  if (timer == STM32_TIM11) {
    rccResetTIM11();
    rccDisableTIM11();
  }
#endif
#ifdef TIM12
  else  if (timer == STM32_TIM12) {
    rccResetTIM12();
    rccDisableTIM12();
  }
#endif
#ifdef TIM13
  else  if (timer == STM32_TIM13) {
    rccResetTIM13();
    rccDisableTIM13();
  }
#endif
#ifdef TIM14
  else  if (timer == STM32_TIM14) {
    rccResetTIM14();
    rccDisableTIM14();
  }
#endif
#ifdef TIM15
  else  if (timer == STM32_TIM15) {
    rccResetTIM15();
    rccDisableTIM15();
  }
#endif
#ifdef TIM16
  else  if (timer == STM32_TIM16) {
    rccResetTIM16();
    rccDisableTIM16();
  }
#endif
#ifdef TIM17
  else  if (timer == STM32_TIM17) {
    rccResetTIM17();
    rccDisableTIM17();
  }
#endif
#ifdef TIM18
  else  if (timer == STM32_TIM18) {
    rccResetTIM18();
    rccDisableTIM18();
  }
#endif
#ifdef TIM19
  else  if (timer == STM32_TIM19) {
    rccResetTIM19();
    rccDisableTIM19();
  }
#endif
  else {
    chSysHalt("not a valid timer");
  }
};


/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
#ifndef STM32_INPUT_CAPTURE_USE_TIM1
#define STM32_INPUT_CAPTURE_USE_TIM1 false
#endif

#ifndef STM32_INPUT_CAPTURE_USE_TIM2
#define STM32_INPUT_CAPTURE_USE_TIM2 false
#endif

#ifndef STM32_INPUT_CAPTURE_USE_TIM3
#define STM32_INPUT_CAPTURE_USE_TIM3 false
#endif

#ifndef STM32_INPUT_CAPTURE_USE_TIM4
#define STM32_INPUT_CAPTURE_USE_TIM4 false
#endif

#ifndef STM32_INPUT_CAPTURE_USE_TIM5
#define STM32_INPUT_CAPTURE_USE_TIM5 false
#endif

#ifndef STM32_INPUT_CAPTURE_USE_TIM8
#define STM32_INPUT_CAPTURE_USE_TIM8 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM1
#define STM32_INPUT_CAPTURE_SHARE_TIM1 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM2
#define STM32_INPUT_CAPTURE_SHARE_TIM2 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM3
#define STM32_INPUT_CAPTURE_SHARE_TIM3 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM4
#define STM32_INPUT_CAPTURE_SHARE_TIM4 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM5
#define STM32_INPUT_CAPTURE_SHARE_TIM5 false
#endif

#ifndef STM32_INPUT_CAPTURE_SHARE_TIM8
#define STM32_INPUT_CAPTURE_SHARE_TIM8 false
#endif

#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM1_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM1_ISR false
#endif

#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM2_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM2_ISR false
#endif


#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM3_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM3_ISR false
#endif


#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM4_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM4_ISR false
#endif


#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM5_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM5_ISR false
#endif


#ifndef STM32_INPUT_CAPTURE_ENABLE_TIM8_ISR
#define STM32_INPUT_CAPTURE_ENABLE_TIM8_ISR false
#endif



#if STM32_INPUT_CAPTURE_USE_TIM1 && (!STM32_INPUT_CAPTURE_SHARE_TIM1) && \
(STM32_GPT_USE_TIM1 || STM32_ICU_USE_TIM1 || STM32_PWM_USE_TIM1)
#error "STM32 INPUT_CAPTURE USE TIM1 but already used by GPT or ICU or PWM"
#endif

#if STM32_INPUT_CAPTURE_USE_TIM2 && (!STM32_INPUT_CAPTURE_SHARE_TIM2) &&					\
(STM32_GPT_USE_TIM2 || STM32_ICU_USE_TIM2 || STM32_PWM_USE_TIM2)
#error "STM32 INPUT_CAPTURE USE TIM2 but already used by GPT or ICU or PWM"
#endif

#if STM32_INPUT_CAPTURE_USE_TIM3 && (!STM32_INPUT_CAPTURE_SHARE_TIM3) &&				\
(STM32_GPT_USE_TIM3 || STM32_ICU_USE_TIM3 || STM32_PWM_USE_TIM3)
#error "STM32 INPUT_CAPTURE USE TIM3 but already used by GPT or ICU or PWM"
#endif

#if STM32_INPUT_CAPTURE_USE_TIM4 && (!STM32_INPUT_CAPTURE_SHARE_TIM4) &&				\
(STM32_GPT_USE_TIM4 || STM32_ICU_USE_TIM4 || STM32_PWM_USE_TIM4)
#error "STM32 INPUT_CAPTURE USE TIM4 but already used by GPT or ICU or PWM"
#endif

#if STM32_INPUT_CAPTURE_USE_TIM5 && (!STM32_INPUT_CAPTURE_SHARE_TIM5) &&				\
(STM32_GPT_USE_TIM5 || STM32_ICU_USE_TIM5 || STM32_PWM_USE_TIM5)
#error "STM32 INPUT_CAPTURE USE TIM5 but already used by GPT or ICU or PWM"
#endif

#if STM32_INPUT_CAPTURE_USE_TIM8 && (!STM32_INPUT_CAPTURE_SHARE_TIM8) &&				\
(STM32_GPT_USE_TIM8 || STM32_ICU_USE_TIM8 || STM32_PWM_USE_TIM8)
#error "STM32 INPUT_CAPTURE USE TIM8 but already used by GPT or ICU or PWM"
#endif


#if STM32_INPUT_CAPTURE_USE_TIM1 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM1_ISR
#if defined(STM32_TIM1_UP_TIM10_HANDLER)
/**
 * @brief   TIM1 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_UP_TIM10_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[0]);

  OSAL_IRQ_EPILOGUE();
}
#elif defined(STM32_TIM1_UP_HANDLER)
OSAL_IRQ_HANDLER(STM32_TIM1_UP_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[0]);

  OSAL_IRQ_EPILOGUE();
}
#else
#error "no handler defined for TIM1"
#endif

#if !defined(STM32_TIM1_CC_HANDLER)
#error "STM32_TIM1_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM1 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM1_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[0]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM1_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM1 */

#if STM32_INPUT_CAPTURE_USE_TIM2 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM2_ISR
#if !defined(STM32_TIM2_HANDLER)
#error "STM32_TIM2_HANDLER not defined"
#endif
/**
 * @brief   TIM2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[1]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM2_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM2 */

#if STM32_INPUT_CAPTURE_USE_TIM3 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM3_ISR
#if !defined(STM32_TIM3_HANDLER)
#error "STM32_TIM3_HANDLER not defined"
#endif
/**
 * @brief   TIM3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[2]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM3_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM3 */

#if STM32_INPUT_CAPTURE_USE_TIM4 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM4_ISR
#if !defined(STM32_TIM4_HANDLER)
#error "STM32_TIM4_HANDLER not defined"
#endif
/**
 * @brief   TIM4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[3]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM4_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM4 */

#if STM32_INPUT_CAPTURE_USE_TIM5 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM5_ISR
#if !defined(STM32_TIM5_HANDLER)
#error "STM32_TIM5_HANDLER not defined"
#endif
/**
 * @brief   TIM5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[4]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM5_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM5 */

#if STM32_INPUT_CAPTURE_USE_TIM8 || defined(__DOXYGEN__)
#if STM32_INPUT_CAPTURE_ENABLE_TIM8_ISR
#if defined(STM32_TIM8_UP_TIM13_HANDLER)
/**
 * @brief   TIM8 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_UP_TIM13_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[5]);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if !defined(STM32_TIM8_CC_HANDLER)
#error "STM32_TIM8_CC_HANDLER not defined"
#endif
/**
 * @brief   TIM8 compare interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  input_capture_lld_serve_interrupt(driverByTimerIndex[5]);

  OSAL_IRQ_EPILOGUE();
}
#endif /* STM32_INPUT_CAPTURE_ENABLE_TIM8_ISR */
#endif /* STM32_INPUT_CAPTURE_USE_TIM8 */

static void input_capture_lld_serve_interrupt(const TimICDriver * const timicp)
{
  uint32_t sr;
  stm32_tim_t * const timer = timicp->config->timer;
  
  sr  = timer->SR;
  sr &= (timer->DIER & STM32_TIM_DIER_IRQ_MASK);
  timer->SR = ~sr;

  if (timicp->channel & TIMIC_CH1) {
    if ((sr & STM32_TIM_SR_CC1IF) != 0)
      _input_capture_isr_invoke_capture_cb(timicp, 0);
  }
  if (timicp->channel & TIMIC_CH2) {
    if ((sr & STM32_TIM_SR_CC2IF) != 0)
      _input_capture_isr_invoke_capture_cb(timicp, 1);
  }
  if (timicp->channel & TIMIC_CH3) {
    if ((sr & STM32_TIM_SR_CC3IF) != 0)
      _input_capture_isr_invoke_capture_cb(timicp, 2);
  }
  if (timicp->channel & TIMIC_CH4) {
    if ((sr & STM32_TIM_SR_CC4IF) != 0)
      _input_capture_isr_invoke_capture_cb(timicp, 3);
  }
  
  if ((sr & STM32_TIM_SR_UIF) != 0)
    _input_capture_isr_invoke_overflow_cb(timicp);
}

static void _input_capture_isr_invoke_capture_cb(const TimICDriver * const timicp, uint32_t channel)
{
  if (timicp->config->capture_cb) {
    timicp->config->capture_cb(timicp, channel, timicp->config->timer->CCR[channel]);
  }
}

static void _input_capture_isr_invoke_overflow_cb(const TimICDriver * const timicp)
{
  if (timicp->config->overflow_cb)
    timicp->config->overflow_cb(timicp);
}
