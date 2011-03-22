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
#define LED_1_BANK 1
#define LED_1_PIN 25

#define LED_2_BANK 1
#define LED_2_PIN 24

#define LED_3_BANK 1
#define LED_3_PIN 23

#define LED_4_BANK 1
#define LED_4_PIN 31

#define LED_5_BANK 1
#define LED_5_PIN 18

#define POWER_SWITCH_LED 5

#define LED_6_BANK 1
#define LED_6_PIN 22

#define CAM_SWITCH_LED 6


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

/* pressure : P0.10 AD1.2 */
#define ANALOG_BARO_PINSEL PINSEL0
#define ANALOG_BARO_PINSEL_VAL 0x03
#define ANALOG_BARO_PINSEL_BIT 20
#define ANALOG_BARO_ADC 1


/* battery */
#define ADC_CHANNEL_VSUPPLY AdcBank0(2)
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2

#define DefaultVoltageOfAdc(adc) (0.0183*adc)


/* baro */
#define ADC_CHANNEL_BARO AdcBank1(2)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_2



/* MS2100 on SSP, IMU connector */
#define MS2100_SS_PIN   28
#define MS2100_SS_IODIR IO1DIR
#define MS2100_SS_IOSET IO1SET
#define MS2100_SS_IOCLR IO1CLR

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

/* PWM5 on CAM connector */
/* P0.21 */
#define PWM0_PINSEL PINSEL1
#define PWM0_PINSEL_VAL 0x01
#define PWM0_PINSEL_BIT 10

/* PWM2 on SPI connector */
/* P0.7 */
#define PWM1_PINSEL PINSEL0
#define PWM1_PINSEL_VAL 0x02
#define PWM1_PINSEL_BIT 14


#define BOARD_HAS_BARO

/*
 * Modem
 */
//#define MODEM_DEVICE Uart1
//#define MODEM_UART_FLAG


#endif /* CONFIG_BOOZ2_V1_0_H */
