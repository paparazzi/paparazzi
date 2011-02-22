#ifndef CONFIG_CSC_V1_0_H
#define CONFIG_CSC_V1_0_H


/* A 60mhz servo driver !!!! this probably has more processing power than apollo13 */
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
#define LED_1_BANK 1
#define LED_1_PIN 18

#define LED_2_BANK 1
#define LED_2_PIN 17

#define LED_3_BANK 1
#define LED_3_PIN 16

#define LED_4_BANK 1
#define LED_4_PIN 31

/* PPM : rc rx on P0.30 (ADC pico-blade pin 3)*/
#define PPM_PINSEL PINSEL1
#define PPM_PINSEL_VAL 0x3
#define PPM_PINSEL_BIT 28

#ifdef RADIO_CONTROL
#define PPM_CRI TIR_CR0I
#define PPM_CCR_CRF TCCR_CR0_F
#define PPM_CCR_CRR TCCR_CR0_R
#define PPM_CCR_CRI TCCR_CR0_I
#define PPM_CR T0CR0
#endif

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN 20

#define SPI1_DRDY_PINSEL PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0


#endif /* CONFIG_CSC_V1_0_H */
