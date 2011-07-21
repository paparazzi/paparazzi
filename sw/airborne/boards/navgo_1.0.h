#ifndef CONFIG_NAVGO_V1_0_H
#define CONFIG_NAVGO_V1_0_H

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

/* Onboard LEDs */
#define LED_1_BANK 1
#define LED_1_PIN 25

#define LED_2_BANK 1
#define LED_2_PIN 24

#define LED_3_BANK 1
#define LED_3_PIN 23

#define LED_4_BANK 1
#define LED_4_PIN 31

/* PPM : rc rx on P0.28 ( CAP0.2 ) */
#define PPM_PINSEL PINSEL1
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 24
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2

/* ADC */

/* battery */
#define ADC_CHANNEL_VSUPPLY AdcBank1(3)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3

#define DefaultVoltageOfAdc(adc) (0.01837*adc)

/* PWM0 (internal PWM5) */
/* P0.21 */
#define PWM0_PINSEL PINSEL1
#define PWM0_PINSEL_VAL 0x01
#define PWM0_PINSEL_BIT 10

/* PWM1 (internal PWM2 */
/* P0.7 */
#define PWM1_PINSEL PINSEL0
#define PWM1_PINSEL_VAL 0x02
#define PWM1_PINSEL_BIT 14

#define BOARD_HAS_BARO

#endif /* CONFIG_UMARIM_V1_0_H */
