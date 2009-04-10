#ifndef CONFIG_CSC_V1_0_H
#define CONFIG_CSC_V1_0_H

/* Master oscillator freq.       */
#define FOSC (12000000) 
/* PLL multiplier                */
#define PLL_MUL (5)         

//#define FOSC (14745600) 
//#define PLL_MUL (4)         

/* CPU clock freq.               */
#define CCLK (FOSC * PLL_MUL) 

/* Peripheral bus speed mask 0x00->4, 0x01-> 1, 0x02 -> 2   */
// 15MHz peripheral bus
//#define PBSD_BITS 0x00    
//#define PBSD_VAL 4

// 30MHz peripheral bus
#define PBSD_BITS 0x02    
#define PBSD_VAL 2

/* Peripheral bus clock freq. */
#define PCLK (CCLK / PBSD_VAL) 

/* Onboard LEDs */
#define LED_1_BANK 0
#define LED_1_PIN 12

#define LED_2_BANK 0
#define LED_2_PIN 13

//#define LED_3_BANK 1
//#define LED_3_PIN 23

//#define LED_4_BANK 1
//#define LED_4_PIN 31



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


#endif /* CONFIG_CSC_V1_0_H */
