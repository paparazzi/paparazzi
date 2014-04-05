#ifndef CONFIG_BOOZ2_V1_0_H
#define CONFIG_BOOZ2_V1_0_H

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
#define LED_1_BANK 1
#define LED_1_PIN 25

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 24

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_BANK 1
#define LED_3_PIN 23

#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_BANK 1
#define LED_4_PIN 31

#define POWER_SWITCH_GPIO GPIOB,GPIO18

#define CAM_SWITCH_GPIO GPIOB,GPIO22

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

/* select P0.13 (ADC_SPARE) as AD1.4 for ADC_0 */
#define ADC_0 AdcBank1(4)
#if USE_ADC_0
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_4
#endif

/* select P0.4 (SCK_0) as AD0.6 for ADC_1 */
#define ADC_1 AdcBank0(6)
#if USE_ADC_1
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_6
#endif

/* select P0.5 (MISO_0) as AD0.7 for ADC_2 */
#define ADC_2 AdcBank0(7)
#if USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_7
#endif

/* select P0.6 (MOSI_0) as AD1.0 for ADC_3 */
#define ADC_3 AdcBank1(0)
#if USE_ADC_3
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_0
#endif

/* battery */
/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank0(2)
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif

#define DefaultVoltageOfAdc(adc) (0.0183*adc)

/* baro */
#define ADC_CHANNEL_BARO AdcBank1(2)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2


/* SPI slaves (IMU connector) */
#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20
#define SPI_SELECT_SLAVE0_PINSEL PINSEL1
#define SPI_SELECT_SLAVE0_PINSEL_BIT 8
#define SPI_SELECT_SLAVE0_PINSEL_VAL 0

#define SPI_SELECT_SLAVE1_PORT 1
#define SPI_SELECT_SLAVE1_PIN 28
#define SPI_SELECT_SLAVE1_PINSEL PINSEL2
#define SPI_SELECT_SLAVE1_PINSEL_BIT 2
#define SPI_SELECT_SLAVE1_PINSEL_VAL 0


/* MAX1168 EOC pin (booz2 imu) */
#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0
#define MAX1168_EOC_VIC_IT VIC_EINT0


/* MS2100 EOC and reset pins (booz2 imu) */
#define MS2100_RESET_PIN   19
#define MS2100_RESET_IODIR IO1DIR
#define MS2100_RESET_IOSET IO1SET
#define MS2100_RESET_IOCLR IO1CLR

#define MS2100_DRDY_PIN  30
#define MS2100_DRDY_PINSEL PINSEL1
#define MS2100_DRDY_PINSEL_BIT 28
#define MS2100_DRDY_PINSEL_VAL 2
#define MS2100_DRDY_EINT 3
#define MS2100_DRDY_VIC_IT VIC_EINT3


/* Servo definition for actruators_pwm driver */

/* BOOZ PWM0 = PWM_SERVO_5 (driver) = PWM5 (lpc)
 * on CAM connector */
#if USE_PWM0
#define PWM_SERVO_5 1
#define SERVO_REG_0 PWMMR5
#endif

/* BOOZ PWM1 = PWM_SERVO_2 (driver) = PWM2 (lpc)
 * on SPI connector */
#if USE_PWM1
#define PWM_SERVO_2 1
#define SERVO_REG_1 PWMMR2
#endif


/* by default enable onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_BOOZ2_V1_0_H */
