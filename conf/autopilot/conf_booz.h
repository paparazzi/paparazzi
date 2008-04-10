#ifndef CONFIG_BOOZ_H
#define CONFIG_BOOZ_H

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


/* PPM : rc rx on P0.6*/
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12

/* SERVOS_4017 */
/* MAT0.1 on P0.5 */
#define SERVO_CLOCK_PIN 5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* reset on P1.23 */
#define SERVO_RESET_PIN 23
#define SERVO_RESET_PINSEL PINSEL2


#define LED_1_BANK 1
#define LED_1_PIN 19

#define LED_2_BANK 1
#define LED_2_PIN 18

#define LED_3_BANK 1
#define LED_3_PIN 17

#define LED_4_BANK 1
#define LED_4_PIN 16

/* ADC */
#define ADC_0 AdcBank0(4)
#ifdef USE_ADC_0
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_4
#endif

#define ADC_1 AdcBank0(1)
#ifdef USE_ADC_1
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif

/* battery on AD1.7 */
#define ADC_BAT AdcBank1(7)
#ifdef USE_ADC_BAT
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif





#endif /* CONFIG_BOOZ_H */
