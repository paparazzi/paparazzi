#ifndef CONFIG_CLASSIX_H
#define CONFIG_CLASSIX_H

/* Master oscillator freq.       */
#define FOSC (12000000)
/* PLL multiplier                */
#define PLL_MUL (5)
/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL)
/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
#define PBSD_BITS 0x00
#define PBSD_VAL 4
/* Peripheral bus clock freq.    */
#define PCLK (CCLK / PBSD_VAL)



#ifdef FBW
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 24

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 31


/* PPM : rc rx on P0.16 : FBW_RC1 connector
#define PPM_PINSEL PINSEL1
#define PPM_PINSEL_VAL 0x03
#define PPM_PINSEL_BIT 0
*/


/* PPM : rc rx on P0.6 : FBW_RC0 connector */
#define PPM_PINSEL PINSEL0
#define PPM_PINSEL_VAL 0x02
#define PPM_PINSEL_BIT 12
#define PPM_CRI TIR_CR2I
#define PPM_CCR_CRF TCCR_CR2_F
#define PPM_CCR_CRR TCCR_CR2_R
#define PPM_CCR_CRI TCCR_CR2_I
#define PPM_CR T0CR2

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

/* SERVOS : 4017 on FBW_RC0 connector (ppm driver) */

/* MAT0.1 on P0.5 */
#define SERVO_CLOCK_PIN 5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10

/* reset on P1.25 */
#define SERVO_RESET_PIN 25
#define SERVO_RESET_PINSEL PINSEL2

/* ADCs */
/* AD0.6 ( on pin P0.4 ) is supply monitoring */
#define USE_AD0_6

#endif /* FBW */



#ifdef AP

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20
#define SPI_SELECT_SLAVE0_PINSEL PINSEL1
#define SPI_SELECT_SLAVE0_PINSEL_BIT 8
#define SPI_SELECT_SLAVE0_PINSEL_VAL 0

#ifndef SITL
/* sitl handles this board as a mono-processor one: unfortunately these
   definitions are already in FBW */

#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 18

#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 19

#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_BANK 1
#define LED_3_PIN 20

#endif /* !SITL */

#endif /* AP */

/* ADC */
/* Definitions for test_adcs.c, to test ADCs on AP ANALOG1 */
#define ADC_0 AdcBank1(5)
#define ADC_1 AdcBank1(6)
#define ADC_2 AdcBank1(7)
#define ADC_3 AdcBank1(4)
#define ADC_4 AdcBank1(3)
#define ADC_5 AdcBank1(2)


#endif /* CONFIG_CLASSIX_H */
