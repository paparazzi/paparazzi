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

/* Onboard leds */
#define LED_1_BANK 1
#define LED_1_PIN 28

#define LED_2_BANK 1
#define LED_2_PIN 19

/* Switch pin */
#define LED_3_BANK 0
#define LED_3_PIN 11

#define POWER_SWITCH_LED 3

/* daugther board leds */
#define LED_4_BANK 0
#define LED_4_PIN 15

#define LED_5_BANK 0
#define LED_5_PIN 13
#define LED_5_INVERTED

#define LED_6_BANK 0
#define LED_6_PIN 12

#define LED_7_BANK 0
#define LED_7_PIN 22




/* P0.5 aka MAT0.1  */
#define SERVO_CLOCK_PIN  5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.23          */
#define SERVO_DATA_PIN  23
/* p1.24          */
#define SERVO_RESET_PIN 24

/* PPM : rc rx on P0.6*/
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12

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

#define ADC_3 AdcBank0(4)
#ifdef USE_ADC_3
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_4
#endif

#define ADC_4 AdcBank1(7)
#ifdef USE_ADC_4
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif

#define ADC_5 AdcBank1(3)
#ifdef USE_ADC_5
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3
#endif

#define ADC_6 AdcBank1(4)
#ifdef USE_ADC_6
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

#define ADC_7 AdcBank1(5)
#ifdef USE_ADC_7
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_5
#endif

#define ADC_CHANNEL_VSUPPLY AdcBank1(6)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6


#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) (0.01787109375*adc)
#endif

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20

#define SPI1_DRDY_PINSEL PINSEL0
#define SPI1_DRDY_PINSEL_BIT  14
#define SPI1_DRDY_PINSEL_VAL   3
#define SPI1_DRDY_EINT         2
#define SPI1_DRDY_VIC_IT       VIC_EINT2

#endif /* CONFIG_TINY_H */
