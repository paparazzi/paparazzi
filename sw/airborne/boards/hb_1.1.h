#ifndef CONFIG_HB_V1_1_H
#define CONFIG_HB_V1_1_H

/* Master oscillator freq.       */
#define FOSC (12000000)

/* PLL multiplier                */
#define PLL_MUL (5)

/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)

/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x02
#define PBSD_VAL 2

/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD_VAL)

/* power switch status */
#define POWER_SWITCH_GPIO GPIOB,GPIO18
#define POWER_SWITCH_2_GPIO GPIOB,GPIO19

/* buzzer and cam switch */
#define BUZZER_GPIO GPIOB,GPIO20
#define CAM_SWITCH_GPIO GPIOB,GPIO25

/* P0.22 aka MAT0.0  */
#define SERVO_CLOCK_PIN  22
#define SERVO_CLOCK_PINSEL PINSEL1
#define SERVO_CLOCK_PINSEL_VAL 0x03
#define SERVO_CLOCK_PINSEL_BIT 12
/* p1.24          */
#define SERVO_RESET_PIN 24

/* PPM : rc rx on P0.16 ( CAP0.2 ) */
#define PPM_PINSEL PINSEL1
#define PPM_PINSEL_VAL 0x03
#define PPM_PINSEL_BIT 0
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2


/* ADC */

/* IR3 */
#define ADC_0 AdcBank0(3)
#if USE_ADC_0
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

/* IR2 */
#define ADC_1 AdcBank0(2)
#if USE_ADC_1
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif

/* IR1 */
#define ADC_2 AdcBank0(1)
#if USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif


/* ADC2 on ANALOG connector */
#define ADC_3 AdcBank1(2)
#if USE_ADC_3
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2
#endif

/* ADC3 on ANALOG connector */
#define ADC_4 AdcBank1(3)
#if USE_ADC_4
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3
#endif

/* ADC4 on ANALOG connector */
#define ADC_5 AdcBank1(4)
#if USE_ADC_5
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank1(6)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif


#define DefaultVoltageOfAdc(adc) (0.032362123*adc)


#endif /* CONFIG_HB_V1_1_H */
