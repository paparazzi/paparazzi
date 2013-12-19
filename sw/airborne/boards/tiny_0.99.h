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

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 28

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 19

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_4015.h"
#define ActuatorDefaultSet(_x,_y) Actuator4015Set(_x,_y)
#define ActuatorsDefaultInit() Actuators4015Init()
#define ActuatorsDefaultCommit() Actuators4015Commit()

/* P0.5 aka MAT0.1  */
#define SERVO_CLOCK_PIN  5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.23          */
#define SERVO_DATA_PIN  20
/* p1.24          */
#define SERVO_RESET_PIN 29

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

#define ADC_3 AdcBank1(7)
#if USE_ADC_3
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif

#define ADC_4 AdcBank1(3)
#if USE_ADC_4
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3
#endif

#define ADC_5 AdcBank1(4)
#if USE_ADC_5
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

#define ADC_6 AdcBank1(5)
#if USE_ADC_6
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_5
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
#define ADC_CHANNEL_VSUPPLY AdcBank1(6)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

#define DefaultVoltageOfAdc(adc) (0.01787109375*adc)

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20
#define SPI_SELECT_SLAVE0_PINSEL PINSEL1
#define SPI_SELECT_SLAVE0_PINSEL_BIT 8
#define SPI_SELECT_SLAVE0_PINSEL_VAL 0

/* MAX1168 EOC pin (e.g. booz2 imu) */
#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0
#define MAX1168_EOC_VIC_IT VIC_EINT0


#endif /* CONFIG_TINY_H */
