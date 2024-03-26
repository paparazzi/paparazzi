#pragma once

#include <ch.h>
#include <hal.h>
#include <string.h>
#include "mcu_periph/hal_stm32_dma.h"



typedef struct {
  DMA_Stream_TypeDef DMA_regs;
  stm32_tim_t	     TIM_regs;
} TimerDmaCache;

void timerDmaCache_cache(TimerDmaCache *tdcp, const DMADriver *fromDma, const  stm32_tim_t *fromTim);
void timerDmaCache_restore(const TimerDmaCache *tdcp, DMADriver *toDma, stm32_tim_t *toTim);



