#ifndef CONFIG_TINY_H
#define CONFIG_TINY_H

#include "types.h"
#include "LPC21xx.h"

/* Master oscillator freq.       */
#define FOSC (14745600) 
/* PLL multiplier                */
#define PLL_MUL (4)         
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL) 
/* Peripheral bus speed divider */
#define PBSD 2    
/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD) 

#define LED_1_BANK 1
#define LED_1_PIN 28

#define LED_2_BANK 1
#define LED_2_PIN 19

#endif /* CONFIG_TINY_H */
