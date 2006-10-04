#ifndef CONFIG_TINY_H
#define CONFIG_TINY_H

#ifdef SITL
/* Dummy definitions: adc are unused anyway */
#define AdcBank0(x) (x)
#define AdcBank1(x) (x)
#endif /* SITL */

/* Master oscillator freq.       */
#define FOSC (12000000) 

/* PLL multiplier                */
#define PLL_MUL (5)         

/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL) 

/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00    
#define PBSD_VAL 4

/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD_VAL) 

#define LED_1_BANK 1
#define LED_1_PIN 28

#define LED_2_BANK 1
#define LED_2_PIN 19

/* P0.5 aka MAT0.1  */
#define SERV1_CLOCK_PIN  5
#define SERV1_CLOCK_PINSEL PINSEL0
#define SERV1_CLOCK_PINSEL_VAL 0x02
#define SERV1_CLOCK_PINSEL_BIT 10
/* p1.23          */
#define SERV1_DATA_PIN  23
/* p1.24          */
#define SERV1_RESET_PIN 24

/* PPM : rc rx on P0.6*/
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12

/* ADC */

#define ADC_0 AdcBank0(3)
#define ADC_1 AdcBank0(2)
#define ADC_2 AdcBank0(1)
#define ADC_3 AdcBank0(4)
#define ADC_4 AdcBank1(7)
#define ADC_5 AdcBank1(3)
#define ADC_6 AdcBank1(4)
#define ADC_7 AdcBank1(5)
#define ADC_CHANNEL_VSUPPLY AdcBank1(6)


#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) (0.01787109375*adc)
#endif

#endif /* CONFIG_TINY_H */
