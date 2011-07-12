#ifndef CONFIG_UMARIM_V1_0_H
#define CONFIG_UMARIM_V1_0_H

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

/* P0.5 aka MAT0.1  */
#define SERVO_CLOCK_PIN  5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.20          */
#define SERVO_RESET_PIN 20

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
#define ADC_CHANNEL_VSUPPLY AdcBank0(2)
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2

#define DefaultVoltageOfAdc(adc) (0.0247*adc)

/* SPI (SSP) */
#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20

#define SPI1_DRDY_PINSEL PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0

#define BOARD_HAS_BARO

#endif /* CONFIG_UMARIM_V1_0_H */
