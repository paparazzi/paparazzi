#ifndef CONFIG_BOOZ2_V1_0_H
#define CONFIG_BOOZ2_V1_0_H

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

/* ADC */

#define ADC_0 AdcBank0(3)
#ifdef USE_ADC_0
#ifndef USE_AD0
#define USE_AD0
#endif
#define USE_AD0_3
#endif

#define ADC_CHANNEL_VSUPPLY AdcBank1(6)
#ifndef USE_AD1
#define USE_AD1
#endif
#define USE_AD1_6

#ifndef VoltageOfAdc
#define VoltageOfAdc(adc) (0.01787109375*adc)
#endif


/* Micromag on SSP, IMU connector */
#define MM_SS_PIN   28
#define MM_SS_IODIR IO1DIR
#define MM_SS_IOSET IO1SET
#define MM_SS_IOCLR IO1CLR

#define MM_RESET_PIN   19
#define MM_RESET_IODIR IO1DIR
#define MM_RESET_IOSET IO1SET
#define MM_RESET_IOCLR IO1CLR

#define MM_DRDY_PIN  30
#define MM_DRDY_PINSEL PINSEL1
#define MM_DRDY_PINSEL_BIT 28
#define MM_DRDY_PINSEL_VAL 2
#define MM_DRDY_EINT 3
#define MM_DRDY_VIC_IT VIC_EINT3

// damit, we have two of them now
//#define POWER_SWITCH_LED 3

/* 4017 servo driver on CAM connector */
/* P0.28 aka MAT0.2  */
//#define SERVO_CLOCK_PIN  28
//#define SERVO_CLOCK_PINSEL PINSEL0
//#define SERVO_CLOCK_PINSEL_VAL 0x02
//#define SERVO_CLOCK_PINSEL_BIT 10
/* p1.23          */
//#define SERVO_DATA_PIN  23
/* p1.24          */
//#define SERVO_RESET_PIN 24


#endif /* CONFIG_BOOZ2_V1_0_H */
