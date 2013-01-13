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

/* current @7V, Tiny2.11, Funjet4, no LED
   PCLK   max   min
   15MHz  99mA  92mA
   30MHz 105mA  98mA
   60MHz 116mA 108mA
*/

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

/* green */
#ifndef USE_LED_1
#define USE_LED_1 1
#endif
#define LED_1_BANK 1
#define LED_1_PIN 17

/* red */
#ifndef USE_LED_2
#define USE_LED_2 1
#endif
#define LED_2_BANK 1
#define LED_2_PIN 16

/* yellow */
#ifndef USE_LED_3
#define USE_LED_3 1
#endif
#define LED_3_BANK 1
#define LED_3_PIN 23

#ifndef USE_LED_4
#define USE_LED_4 1
#endif
#define LED_4_BANK 1
#define LED_4_PIN 18

#define POWER_SWITCH_LED 4

#ifndef USE_LED_5
#define USE_LED_5 1
#endif
#define LED_5_BANK 1
#define LED_5_PIN 22

#define CAM_SWITCH_LED 5

#ifndef USE_LED_6
#define USE_LED_6 1
#endif
#define LED_6_BANK 1
#define LED_6_PIN 21

#define GPS_RESET 6

#define Configure_GPS_RESET_Pin() LED_INIT(GPS_RESET)
#define Set_GPS_RESET_Pin_LOW() LED_ON(GPS_RESET)
#define Open_GPS_RESET_Pin() ClearBit(LED_DIR(GPS_RESET), LED_PIN(GPS_RESET))

/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_4017.h"
#define ActuatorDefaultSet(_x,_y) Actuator4017Set(_x,_y)
#define ActuatorsDefaultInit() Actuators4017Init()
#define ActuatorsDefaultCommit() Actuators4017Commit()

/* P0.5 aka MAT0.1  */
#define SERVO_CLOCK_PIN  5
#define SERVO_CLOCK_PINSEL PINSEL0
#define SERVO_CLOCK_PINSEL_VAL 0x02
#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.20          */
#define SERVO_RESET_PIN 20

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

#define ADC_0 AdcBank1(6)
#ifdef USE_ADC_0
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6
#endif

#define ADC_1 AdcBank1(7)
#ifdef USE_ADC_1
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_7
#endif


#define ADC_2 AdcBank0(4)
#ifdef USE_ADC_2
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_4
#endif

#define ADC_3 AdcBank0(6)
#ifdef USE_ADC_3
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_6
#endif

#define ADC_4 AdcBank0(3)
#ifdef USE_ADC_4
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

#define ADC_5 AdcBank0(2)
#ifdef USE_ADC_5
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_2
#endif

#define ADC_6 AdcBank0(1)
#ifdef USE_ADC_6
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_1
#endif

#define ADC_7 AdcBank1(3)
#ifdef USE_ADC_7
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_3
#endif

/* allow to define ADC_CHANNEL_VSUPPLY in the airframe file*/
#ifndef ADC_CHANNEL_VSUPPLY
#define ADC_CHANNEL_VSUPPLY AdcBank1(5)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_5
#endif


#define DefaultVoltageOfAdc(adc) (0.01787109375*adc)

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20
#define SPI_SELECT_SLAVE0_PINSEL PINSEL1
#define SPI_SELECT_SLAVE0_PINSEL_BIT 8
#define SPI_SELECT_SLAVE0_PINSEL_VAL 0

#define SPI1_DRDY_PINSEL PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0


/* MAX1168 EOC pin (e.g. booz2 imu) */
#define MAX1168_EOC_PIN 16
#define MAX1168_EOC_PINSEL PINSEL1
#define MAX1168_EOC_PINSEL_BIT 0
#define MAX1168_EOC_PINSEL_VAL 1
#define MAX1168_EOC_EINT 0
#define MAX1168_EOC_VIC_IT VIC_EINT0

#endif /* CONFIG_TINY_H */
