#ifndef CONFIG_MOTOR_BENCH_H
#define CONFIG_MOTOR_BENCH_H

/* Master oscillator freq.       */
#define FOSC (12000000) 
/* PLL multiplier                */
#define PLL_MUL (5)         
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL) 
/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00    
#define PBSD_VAL 4
/* Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD_VAL) 

#define LED_1_BANK 0
#define LED_1_PIN 12

#define LED_2_BANK 0
#define LED_2_PIN 13

#endif /* CONFIG_MOTOR_BENCH_H */
