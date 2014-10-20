#ifndef CONFIG_HBMINI_V1_0_H
#define CONFIG_HBMINI_V1_0_H

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
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 0
#define LED_1_PIN 22

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 28

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_BANK 1
#define LED_3_PIN 29

#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_BANK 1
#define LED_4_PIN 30

/* PPM : rc rx on P0.28 ( CAP0.2 ) */
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2

/* ADC */

/* not compatible with PWM1 */
#define ADC_0 AdcBank1(6)
#if USE_ADC_0
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

/* battery */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank1(6)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

#define DefaultVoltageOfAdc(adc) (0.03385*adc)

/* SPI (SSP) */
#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20
#define SPI_SELECT_SLAVE0_PINSEL PINSEL1
#define SPI_SELECT_SLAVE0_PINSEL_BIT 8
#define SPI_SELECT_SLAVE0_PINSEL_VAL 0

//#define SPI_SELECT_SLAVE1_PORT 1
//#define SPI_SELECT_SLAVE1_PIN 19

#define SPI1_DRDY_PINSEL PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0


/* MAX1168 EOC pin (booz2 imu) */
#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0
#define MAX1168_EOC_VIC_IT VIC_EINT0

/* Servo definition for actruators_pwm driver */

/* HBMINI PWM0 = PWM_SERVO_2 (driver) = PWM2 (lpc) */
#if USE_PWM0
#define PWM_SERVO_2 1
#define SERVO_REG_0 PWMMR2
#endif

/* HBMINI PWM1 = PWM_SERVO_5 (driver) = PWM5 (lpc) */
#if USE_PWM1
#define PWM_SERVO_5 1
#define SERVO_REG_1 PWMMR5
#endif


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_HBMINI_V1_0_H */
