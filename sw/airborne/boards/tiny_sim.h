#ifndef CONFIG_TINY_H
#define CONFIG_TINY_H

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

/* p0.21 aka PWM5 */
#define SERV0_CLOCK_PIN 21
#define SERV0_CLOCK_PINSEL PINSEL1
#define SERV0_CLOCK_PINSEL_VAL 0x01
#define SERV0_CLOCK_PINSEL_BIT 10
/* p1.20          */
#define SERV0_DATA_PIN  20
/* p1.21          */
#define SERV0_RESET_PIN 21

/* P0.7 aka PWM2  */
#define SERV1_CLOCK_PIN  7
#define SERV1_CLOCK_PINSEL PINSEL0
#define SERV1_CLOCK_PINSEL_VAL 0x02
#define SERV1_CLOCK_PINSEL_BIT 14
/* p1.30          */
#define SERV1_DATA_PIN  30
/* p1.29          */
#define SERV1_RESET_PIN 29

/* PPM : rc rx on P0.6*/
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2

/* ADC */

#define ADC_0 AdcBank0(3)
#ifdef USE_ADC_0
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

#define ADC_1 AdcBank0(2)
#ifdef USE_ADC_1
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif


#define ADC_2 AdcBank0(1)
#ifdef USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif

#define ADC_3 AdcBank0(6)
#ifdef USE_ADC_3
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_6
#endif


#define ADC_4 AdcBank1(3)
#ifdef USE_ADC_4
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3
#endif

#define ADC_5 AdcBank1(4)
#ifdef USE_ADC_5
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

#define ADC_6 AdcBank1(5)
#ifdef USE_ADC_6
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_5
#endif

#define ADC_7 AdcBank1(2)
#ifdef USE_ADC_7
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2
#endif

/* #define ADC_3 AdcBank1(7) Used for VSUPPLY */
#define ADC_CHANNEL_VSUPPLY AdcBank1(7)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7


#define DefaultVoltageOfAdc(adc) (0.01787109375*adc)


#endif /* CONFIG_TINY_H */
