/* board definition file for Sparkfun Logomatic V2.6 data logger */

#ifndef CONFIG_LOGOMATIC_H
#define CONFIG_LOGOMATIC_H

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

/* PCLK needs to run @ 30MHz for USB */
#define USE_USB_HIGH_PCLK

#ifdef USE_USB_HIGH_PCLK
/* Peripheral bus speed mask  0x00-> 4, 0x01-> 1, 0x02-> 2   */
/* change both PBSD_BITS/VAL     15MHz,    60MHz,    30MHz   */
#define PBSD_BITS 0x02
#define PBSD_VAL 2
#else
/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00
#define PBSD_VAL 4
#endif

/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD_VAL)

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 16

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 0
#define LED_2_PIN 2

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_BANK 0
#define LED_3_PIN 11

#define POWER_SWITCH_GPIO GPIOB,GPIO17

#define CAM_SWITCH_GPIO GPIOB,GPIO18

#define GPS_RESET_GPIO GPIOB,GPIO19

#define Configure_GPS_RESET_Pin() gpio_setup_output(GPS_RESET_GPIO)
#define Set_GPS_RESET_Pin_LOW() gpio_clear(GPS_RESET_GPIO)
#define Open_GPS_RESET_Pin() gpio_setup_input(GPS_RESET_GPIO)

/* P0.29 aka MAT0.3 (P2) */
#define SERVO_CLOCK_PIN  29
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.20          */
#define SERVO_RESET_PIN 20

/* PPM : rc rx on P0.6 aka CAP0.0 (P1) */
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
#if USE_ADC_0
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

#define ADC_1 AdcBank0(2)
#if USE_ADC_1
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif


#define ADC_2 AdcBank0(1)
#if USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif

#define ADC_3 AdcBank0(4)
#if USE_ADC_3
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_4
#endif

#define ADC_4 AdcBank1(7)
#if USE_ADC_4
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif

#define ADC_5 AdcBank1(6)
#if USE_ADC_5
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

#define ADC_6 AdcBank1(2)
#if USE_ADC_6
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2
#endif

#define ADC_7 AdcBank1(2)
#if USE_ADC_7
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank1(4)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif


#define DefaultVoltageOfAdc(adc) (0.068505859375*adc)

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20

#define SPI1_DRDY_PINSEL PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0

/* micro SD connected to SPI0 */
#define SPI_CHANNEL 0

/* STOP button P0.3 */
#define LOG_STOP_KEY 3

#endif /* CONFIG_LOGOMATIC_H */

