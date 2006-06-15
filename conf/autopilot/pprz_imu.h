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

#endif /* CONFIG_PPRZ_IMU_H */
