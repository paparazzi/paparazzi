#ifndef CONFIG_OLIMEX_H2148_H
#define CONFIG_OLIMEX_H2148_H

/* Master oscillator freq.       */
#define FOSC (12000000)
/* PLL multiplier                */
#define PLL_MUL (5)
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)
/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x02
#define PBSD_VAL 2
/* Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD_VAL)

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 24

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 23

#endif /* CONFIG_OLIMEX_H2148_H */
