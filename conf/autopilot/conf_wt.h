#ifndef CONFIG_WT_H
#define CONFIG_WT_H

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

#define LED_1_BANK 1
#define LED_1_PIN 24

#define SPI_SELECT_SLAVE0_PORT 0
#define SPI_SELECT_SLAVE0_PIN  20

#define SPI1_DRDY_PINSEL       PINSEL1
#define SPI1_DRDY_PINSEL_BIT   0
#define SPI1_DRDY_PINSEL_VAL   1
#define SPI1_DRDY_EINT         0
#define SPI1_DRDY_VIC_IT       VIC_EINT0


#endif /* CONFIG_WT_H */
