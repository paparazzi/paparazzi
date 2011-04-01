#ifndef LPC21_MCU_PERIPH_DAC_ARCH_H
#define LPC21_MCU_PERIPH_DAC_ARCH_H

#include "std.h"
#include "LPC21xx.h"

static inline void DACSet(uint16_t x) {
  DACR = x << 6;
}


#endif /* LPC21_MCU_PERIPH_DAC_ARCH_H */
