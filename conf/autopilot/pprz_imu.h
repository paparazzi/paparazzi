#ifndef CONFIG_PPRZ_IMU_H
#define CONFIG_PPRZ_IMU_H

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

#define ADC_CHANNEL_AX  2
#define ADC_CHANNEL_AY  1
#define ADC_CHANNEL_AZ  3
#define ADC_CHANNEL_BAT 4

//#define LED_1_BANK 0
//#define LED_1_PIN 2
#define LED_1_BANK 1
#define LED_1_PIN  24

//#define LED_2_BANK 0
//#define LED_2_PIN 3


#define MM_SS_PIN 20
#define MM_SS_IODIR IO0DIR
#define MM_SS_IOSET IO0SET
#define MM_SS_IOCLR IO0CLR

#define MM_RESET_PIN 21
#define MM_RESET_IODIR IO1DIR
#define MM_RESET_IOSET IO1SET
#define MM_RESET_IOCLR IO1CLR

#define MM_DRDY_PIN 15
#define MM_DRDY_PINSEL PINSEL0
#define MM_DRDY_PINSEL_BIT 30
#define MM_DRDY_PINSEL_VAL 2
#define MM_DRDY_EINT 2
#define MM_DRDY_VIC_IT VIC_EINT2



#endif /* CONFIG_PPRZ_IMU_H */
