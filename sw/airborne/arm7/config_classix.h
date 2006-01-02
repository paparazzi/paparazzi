#ifndef CONFIG_CLASSIX_H
#define CONFIG_CLASSIX_H

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

#ifdef FBW
#define LED_1_BANK 1
#define LED_1_PIN 24

#define LED_2_BANK 1
#define LED_2_PIN 31
#endif /* FBW */

#ifdef AP
#define LED_1_BANK 1
#define LED_1_PIN 18

#define LED_2_BANK 1
#define LED_2_PIN 19
#endif /* AP */

#endif /* CONFIG_CLASSIX_H */
