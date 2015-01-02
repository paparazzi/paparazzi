#ifndef LPC2138_H
#define LPC2138_H

#include "inttypes.h"


/* Pin Connect Block */
#define PINSEL0        (*((volatile uint32_t *) 0xE002C000))
#define PINSEL1        (*((volatile uint32_t *) 0xE002C004))
#define PINSEL2        (*((volatile uint32_t *) 0xE002C014))


/* General Purpose Input/Output (GPIO) */
#define IOPIN0         (*((volatile uint32_t *) 0xE0028000))
#define IOSET0         (*((volatile uint32_t *) 0xE0028004))
#define IODIR0         (*((volatile uint32_t *) 0xE0028008))
#define IOCLR0         (*((volatile uint32_t *) 0xE002800C))
#define IOPIN1         (*((volatile uint32_t *) 0xE0028010))
#define IOSET1         (*((volatile uint32_t *) 0xE0028014))
#define IODIR1         (*((volatile uint32_t *) 0xE0028018))
#define IOCLR1         (*((volatile uint32_t *) 0xE002801C))


/* Memory Accelerator Module (MAM) */
#define MAMCR          (*((volatile uint8_t *) 0xE01FC000))
#define MAMTIM         (*((volatile uint8_t *) 0xE01FC004))
#define MEMMAP         (*((volatile uint8_t *) 0xE01FC040))


/* Phase Locked Loop (PLL) */
#define PLLCON         (*((volatile uint8_t*) 0xE01FC080))
#define PLLCON_bit     (*(struct PLLCON_reg*) 0xE01FC080)
struct PLLCON_reg {
  unsigned char PLLE: 1;
  unsigned char PLLC: 1;
};

#define PLLCFG         (*((volatile uint8_t*) 0xE01FC084))
#define PLLCFG_bit     (*(struct PLLCFG_reg*) 0xE01FC084)
struct PLLCFG_reg {
  unsigned char MSEL: 5;
  unsigned char PSEL: 2;
};

#define PLLSTAT        (*((volatile uint16_t*) 0xE01FC088))
#define PLLSTAT_bit    (*(struct PLLSTAT_reg*) 0xE01FC088)
struct PLLSTAT_reg {
  unsigned short MSEL: 5;
  unsigned short PSEL: 2;
  unsigned short _: 1;
  unsigned short PLLE: 1;
  unsigned short PLLC: 1;
  unsigned short PLOCK: 1;
};

#define PLLFEED        (*((volatile uint8_t*) 0xE01FC08C))
#define PLLFEED_bit    (*(struct PLLFEED_reg*) 0xE01FC08C)
struct PLLFEED_reg {
  unsigned char FEED: 8;
};


/* Power Control */
#define PCON           (*((volatile uint8_t *)  0xE01FC0C0))
#define PCON_bit       (*(struct PCON_reg*)      0xE01FC0C0)
struct PCON_reg {
  uint8_t IDL: 1;
  uint8_t PD: 1;
  uint8_t PDBOD: 2;
};
#define PCONP          (*((volatile uint32_t *) 0xE01FC0C4))
#define PCONP_bit      (*(struct PCONP_reg*)     0xE01FC0C4)
struct PCONP_reg {
  uint32_t _1: 1;
  uint32_t PCTIM0: 1;
  uint32_t PCTIM1: 1;
  uint32_t PCUART0: 1;
  uint32_t PCUART1: 1;
  uint32_t PCPWM0: 1;
  uint32_t _2: 1;
  uint32_t PCI2C0: 1;
  uint32_t PCSPI0: 1;
  uint32_t PCRTC: 1;
  uint32_t PCSPI1: 1;
  uint32_t _3: 1;
  uint32_t PCAD0: 1;
  uint32_t _4: 6;
  uint32_t PCI2C1: 1;
  uint32_t PCAD1: 1;
  uint32_t _5: 11;
};

/* VPB Divider */
#define VPBDIV         (*((volatile uint8_t *)  0xE01FC100))
#define VPBDIV_bit     (*(struct VPBDIV_reg*)    0xE01FC100)
struct VPBDIV_reg {
  uint8_t VPB_DIV: 2;
  uint8_t _: 6;
};

/* External Interrupts */
#define EXTINT         (*((volatile uint8_t *)  0xE01FC140))
#define INTWAKE        (*((volatile uint8_t *)  0xE01FC144))
#define  EXTWAKE       (*((volatile uint8_t *)  0xE01FC144))
#define EXTMODE        (*((volatile uint8_t *)  0xE01FC148))
#define EXTPOLAR       (*((volatile uint8_t *)  0xE01FC14C))

/* Reset */
#define RSIR           (*((volatile uint8_t *)  0xE01FC180))
#define RSIR_bit       (*(struct RSIR_reg*)      0xE01FC180)
struct RSIR_reg {
  uint8_t POR: 1;
  uint8_t EXTR: 1;
  uint8_t WDTR: 1;
  uint8_t BODR: 1;
  uint8_t _: 4;
};

/* Code Security / Debugging */
#define CSPR           (*((volatile uint8_t *)  0xE01FC184))

/* Timer 0 */
#define T0IR           (*((volatile uint8_t *)  0xE0004000))
#define T0IR_bit       (*(struct T0IR_reg*)     0xE0004000)
struct T0IR_reg {
  uint8_t MR0I: 1;
  uint8_t MR1I: 1;
  uint8_t MR2I: 1;
  uint8_t MR3I: 1;
  uint8_t CR0I: 1;
  uint8_t CR1I: 1;
  uint8_t CR2I: 1;
  uint8_t CR3I: 1;
};

#define T0TCR          (*((volatile uint8_t *)  0xE0004004))
#define T0TCR_bit      (*(struct T0TCR_reg*)    0xE0004004)
struct T0TCR_reg {
  uint8_t CE: 1;
  uint8_t CR: 1;
  uint8_t _: 6;
};

#define T0TC           (*((volatile uint32_t *) 0xE0004008))
#define T0PR           (*((volatile uint32_t *) 0xE000400C))
#define T0PC           (*((volatile uint32_t *) 0xE0004010))

#define T0MCR          (*((volatile uint16_t *) 0xE0004014))
#define T0MCR_bit      (*(struct T0MCR_reg*)    0xE0004014)
struct T0MCR_reg {
  uint32_t MR0I: 1;
  uint32_t MR0R: 1;
  uint32_t MR0S: 1;
  uint32_t MR1I: 1;
  uint32_t MR1R: 1;
  uint32_t MR1S: 1;
  uint32_t MR2I: 1;
  uint32_t MR2R: 1;
  uint32_t MR2S: 1;
  uint32_t MR3I: 1;
  uint32_t MR3R: 1;
  uint32_t MR3S: 1;
  uint32_t _: 4;
};


#define T0MR0          (*((volatile uint32_t *) 0xE0004018))
#define T0MR1          (*((volatile uint32_t *) 0xE000401C))
#define T0MR2          (*((volatile uint32_t *) 0xE0004020))
#define T0MR3          (*((volatile uint32_t *) 0xE0004024))
#define T0CCR          (*((volatile uint32_t *) 0xE0004028))
#define T0CR0          (*((volatile uint32_t *) 0xE000402C))
#define T0CR1          (*((volatile uint32_t *) 0xE0004030))
#define T0CR2          (*((volatile uint32_t *) 0xE0004034))
#define T0CR3          (*((volatile uint32_t *) 0xE0004038))
#define T0EMR          (*((volatile uint32_t *) 0xE000403C))
#define T0CTCR         (*((volatile uint32_t *) 0xE0004070))


/* Timer 1 */
#define T1IR           (*((volatile uint32_t *) 0xE0008000))
#define T1TCR          (*((volatile uint32_t *) 0xE0008004))
#define T1TC           (*((volatile uint32_t *) 0xE0008008))
#define T1PR           (*((volatile uint32_t *) 0xE000800C))
#define T1PC           (*((volatile uint32_t *) 0xE0008010))
#define T1MCR          (*((volatile uint32_t *) 0xE0008014))
#define T1MR0          (*((volatile uint32_t *) 0xE0008018))
#define T1MR1          (*((volatile uint32_t *) 0xE000801C))
#define T1MR2          (*((volatile uint32_t *) 0xE0008020))
#define T1MR3          (*((volatile uint32_t *) 0xE0008024))
#define T1CCR          (*((volatile uint32_t *) 0xE0008028))
#define T1CR0          (*((volatile uint32_t *) 0xE000802C))
#define T1CR1          (*((volatile uint32_t *) 0xE0008030))
#define T1CR2          (*((volatile uint32_t *) 0xE0008034))
#define T1CR3          (*((volatile uint32_t *) 0xE0008038))
#define T1EMR          (*((volatile uint32_t *) 0xE000803C))
#define T1CTCR         (*((volatile uint32_t *) 0xE0008070))


/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define U0LCR          (*((volatile uint8_t *) 0xE000C00C))
#define U0LCR_bit      (*(struct U0LCR_reg*) 0xE000C00C)
struct U0LCR_reg {
  unsigned char WLS: 2;
  unsigned char SBS: 1;
  unsigned char PE: 1;
  unsigned char PS: 2;
  unsigned char BC: 1;
  unsigned char DLAB: 1;
};
#define U0DLL          (*((volatile unsigned char *) 0xE000C000))
#define U0DLM          (*((volatile unsigned char *) 0xE000C004))

/* I2C Interface 0 */
#define I2C0CONSET      (*((volatile uint8_t *) 0xE001C000))
#define I2C0CONSET_bit  (*(struct I2CONSET_reg*) 0xE001C000)
struct I2CONSET_reg {
  uint8_t _1: 2;
  uint8_t AA: 1;
  uint8_t SI: 1;
  uint8_t STO: 1;
  uint8_t STA: 1;
  uint8_t I2EN: 1;
  uint8_t _2: 1;
};
#define I2C0STAT        (*((volatile uint8_t *) 0xE001C004))
#define I2C0STAT_bit    (*(struct I2STAT_reg*)   0xE001C018)
struct I2STAT_reg {
  uint8_t _: 3;
  uint8_t STATUS: 5;
};
#define I2C0DAT         (*((volatile uint8_t *) 0xE001C008))
#define I2C0ADR         (*((volatile uint8_t *) 0xE001C00C))
#define I2C0ADR_bit     (*(struct I2ADR_reg*)    0xE001C018)
struct I2ADR_reg {
  uint8_t GC: 2;
  uint8_t ADDRESS: 7;
};
#define I2C0SCLH        (*((volatile uint16_t *) 0xE001C010))
#define I2C0SCLL        (*((volatile uint16_t *) 0xE001C014))
#define I2C0CONCLR      (*((volatile uint8_t *)  0xE001C018))
#define I2C0CONCLR_bit  (*(struct I2CONCLR_reg*)  0xE001C018)
struct I2CONCLR_reg {
  uint8_t _1: 2;
  uint8_t AAC: 1;
  uint8_t SIC: 1;
  uint8_t _2: 1;
  uint8_t STAC: 1;
  uint8_t I2ENC: 1;
  uint8_t _3: 1;
};

/* I2C Interface 1 */
#define I2C1CONSET      (*((volatile uint8_t *)  0xE005C000))
#define I2C1CONSET_bit  (*(struct I2CONSET_reg*)  0xE005C000)
#define I2C1STAT        (*((volatile uint8_t *)  0xE005C004))
#define I2C1STAT_bit    (*(struct I2STAT_reg*)    0xE005C004)
#define I2C1DAT         (*((volatile uint8_t *)  0xE005C008))
#define I2C1ADR         (*((volatile uint8_t *)  0xE005C00C))
#define I2C1ADR_bit     (*(struct I2ADR_reg*)     0xE005C00C)
#define I2C1SCLH        (*((volatile uint16_t *) 0xE005C010))
#define I2C1SCLL        (*((volatile uint16_t *) 0xE005C014))
#define I2C1CONCLR      (*((volatile uint8_t *)  0xE005C018))
#define I2C1CONCLR_bit  (*(struct I2CONCLR_reg*)  0xE005C018)

/* SPI0 (Serial Peripheral Interface 0) */
#define S0SPCR         (*((volatile uint16_t *) 0xE0020000))
#define S0SPCR_bit     (*(struct S0SPCR_reg*)    0xE0020000)
struct S0SPCR_reg {
  uint16_t _1: 2;
  uint16_t BITENABLE: 1;
  uint16_t CPHA: 1;
  uint16_t CPOL: 1;
  uint16_t MSTR: 1;
  uint16_t LSBF: 1;
  uint16_t SPIE: 1;
  uint16_t BITS: 4;
  uint16_t _2: 4;
};
#define S0SPSR         (*((volatile uint8_t *)  0xE0020004))
#define S0SPSR_bit     (*(struct S0SPSR_reg*)    0xE0020004)
struct S0SPSR_reg {
  uint8_t _: 3;
  uint8_t BRT: 1;
  uint8_t MODF: 1;
  uint8_t ROVR: 1;
  uint8_t WCOL: 1;
  uint8_t SPIF: 1;
};
#define S0SPDR         (*((volatile uint8_t *)  0xE0020008))
#define S0SPCCR        (*((volatile uint8_t *)  0xE002000C))
#define S0SPINT        (*((volatile uint8_t *)  0xE002001C))
#define S0SPINT_bit     (*(struct S0SPINT_reg*)  0xE002001C)
struct S0SPINT_bit_reg {
  uint8_t SPIINT: 1;
};

/* SSP Controller */
#define SSPCR0         (*((volatile uint16_t *) 0xE0068000))
#define SSPCR0_bit     (*(struct SSPCR0_reg*)    0xE0068000)
struct SSPCR0_reg {
  uint16_t DSS: 4;
  uint16_t FRF: 2;
  uint16_t CPOL: 1;
  uint16_t CPHA: 1;
  uint16_t SCR: 8;
};
#define SSPCR1         (*((volatile uint8_t *)  0xE0068004))
#define SSPCR1_bit     (*(struct SSPCR1_reg*)    0xE0068004)
struct SSPCR1_reg {
  uint8_t LBM: 1;
  uint8_t SSE: 1;
  uint8_t MS: 1;
  uint8_t SOD: 1;
  uint8_t _: 4;
};
#define SSPDR          (*((volatile uint16_t *) 0xE0068008))
#define SSPSR          (*((volatile uint8_t *)  0xE006800C))
#define SSPSR_bit      (*(struct SSPSR_reg*)     0xE006800C)
struct SSPSR_reg {
  uint8_t TFE: 1;
  uint8_t TNF: 1;
  uint8_t RNE: 1;
  uint8_t RFF: 1;
  uint8_t BSY: 1;
  uint8_t _: 3;
};
#define SSPCPSR        (*((volatile uint8_t *)  0xE0068010))
#define SSPIMSC        (*((volatile uint8_t *)  0xE0068014))
#define SSPIMSC_bit    (*(struct SSPIMSC_reg*)   0xE0068014)
struct SSPIMSC_reg {
  uint8_t RORIM: 1;
  uint8_t RTIM: 1;
  uint8_t RXIM: 1;
  uint8_t TXIM: 1;
  uint8_t _: 4;
};
#define SSPRIS         (*((volatile uint8_t *)  0xE0068018))
#define SSPRIS_bit     (*(struct SSPRIS_reg*)    0xE0068018)
struct SSPRIS_reg {
  uint8_t RORRIS: 1;
  uint8_t RTRIS: 1;
  uint8_t RXRIS: 1;
  uint8_t TXRIS: 1;
  uint8_t _: 4;
};
#define SSPMIS         (*((volatile uint8_t *)  0xE006801C))
#define SSPMIS_bit     (*(struct SSPMIS_reg*)    0xE006801C)
struct SSPMIS_reg {
  uint8_t RORMIS: 1;
  uint8_t RTMIS: 1;
  uint8_t RXMIS: 1;
  uint8_t TXMIS: 1;
  uint8_t _: 4;
};
#define SSPICR         (*((volatile uint8_t *)  0xE0068020))
#define SSPICR_bit     (*(struct SSPICR_reg*)    0xE0068020)
struct SSPICR_reg {
  uint8_t RORIC: 1;
  uint8_t RTIC: 1;
  uint8_t _: 6;
};

/* Watchdog */
#define WDMOD          (*((volatile uint8_t *)  0xE0000000))
#define WDTC           (*((volatile uint32_t *) 0xE0000004))
#define WDFEED         (*((volatile uint8_t *)  0xE0000008))
#define WDTV           (*((volatile uint32_t *) 0xE000000C))

/* Real Time Clock */
#define ILR            (*((volatile uint8_t *)  0xE0024000))
#define CTC            (*((volatile uint16_t *) 0xE0024004))
#define CCR            (*((volatile uint8_t *)  0xE0024008))
#define CIIR           (*((volatile uint8_t *)  0xE002400C))
#define AMR            (*((volatile uint8_t *)  0xE0024010))
#define CTIME0         (*((volatile uint32_t *) 0xE0024014))
#define CTIME1         (*((volatile uint32_t *) 0xE0024018))
#define CTIME2         (*((volatile uint32_t *) 0xE002401C))
#define SEC            (*((volatile uint8_t *)  0xE0024020))
#define MIN            (*((volatile uint8_t *)  0xE0024024))
#define HOUR           (*((volatile uint8_t *)  0xE0024028))
#define DOM            (*((volatile uint8_t *)  0xE002402C))
#define DOW            (*((volatile uint8_t *)  0xE0024030))
#define DOY            (*((volatile uint16_t *) 0xE0024034))
#define MONTH          (*((volatile uint8_t *)  0xE0024038))
#define YEAR           (*((volatile uint16_t *) 0xE002403C))
#define ALSEC          (*((volatile uint8_t *)  0xE0024060))
#define ALMIN          (*((volatile uint8_t *)  0xE0024064))
#define ALHOUR         (*((volatile uint8_t *)  0xE0024068))
#define ALDOM          (*((volatile uint8_t *)  0xE002406C))
#define ALDOW          (*((volatile uint8_t *)  0xE0024070))
#define ALDOY          (*((volatile uint16_t *) 0xE0024074))
#define ALMON          (*((volatile uint8_t *)  0xE0024078))
#define ALYEAR         (*((volatile uint16_t *) 0xE002407C))
#define PREINT         (*((volatile uint16_t *) 0xE0024080))
#define PREFRAC        (*((volatile uint16_t *) 0xE0024084))

/* A/D Converter 0 (AD0) */
#define AD0CR          (*((volatile uint32_t *) 0xE0034000))
#define AD0CR_bit      (*(struct ADCR_reg*)      0xE0034000)
struct ADCR_reg {
  uint32_t SEL: 8;
  uint32_t CLKDIV: 8;
  uint32_t BURST: 1;
  uint32_t CLKS: 3;
  uint32_t _1: 1;
  uint32_t PDN: 1;
  uint32_t _2: 2;
  uint32_t START: 3;
  uint32_t EDGE: 1;
  uint32_t _3: 4;
};
#define AD0DR          (*((volatile uint32_t *) 0xE0034004))
#define AD0DR_bit      (*(struct ADDR_reg*)      0xE0034004)
struct ADDR_reg {
  uint32_t _1: 6;
  uint32_t Vref: 10;
  uint32_t _2: 8;
  uint32_t CHN: 3;
  uint32_t _3: 3;
  uint32_t OVERRUN: 1;
  uint32_t DONE: 1;
};
#define ADGSR          (*((volatile uint32_t *) 0xE0034008))
#define ADGSR_bit      (*(struct ADGSR_reg*)     0xE0034008)
struct ADGSR_reg {
  uint32_t _1: 16;
  uint32_t BURST: 1;
  uint32_t _2: 7;
  uint32_t START: 3;
  uint32_t EDGE: 1;
  uint32_t _3: 4;
};

/* A/D Converter 1 (AD1) */
#define AD1CR          (*((volatile uint32_t *) 0xE0060000))
#define AD1CR_bit      (*(struct ADCR_reg*)      0xE0034000)
#define AD1DR          (*((volatile uint32_t *) 0xE0060004))
#define AD1DR_bit      (*(struct ADDR_reg*)      0xE0060004)

/* D/A Converter */
#define DACR           (*((volatile uint32_t *) 0xE006C000))
#define DACR_bit       (*(struct DACR_reg*)      0xE006C000)
struct DACR_reg {
  uint32_t _: 6;
  uint32_t VALUE: 10;
  uint32_t BIAS: 1;
};

/* Vectored Interrupt Controller (VIC) */
#define VICIRQStatus   (*((volatile uint32_t *) 0xFFFFF000))
#define VICFIQStatus   (*((volatile uint32_t *) 0xFFFFF004))
#define VICRawIntr     (*((volatile uint32_t *) 0xFFFFF008))
#define VICIntSelect   (*((volatile uint32_t *) 0xFFFFF00C))
#define VICIntEnable   (*((volatile uint32_t *) 0xFFFFF010))
#define VICIntEnClr    (*((volatile uint32_t *) 0xFFFFF014))
#define VICSoftInt     (*((volatile uint32_t *) 0xFFFFF018))
#define VICSoftIntClr  (*((volatile uint32_t *) 0xFFFFF01C))
#define VICProtection  (*((volatile uint32_t *) 0xFFFFF020))
#define VICVectAddr    (*((volatile uint32_t *) 0xFFFFF030))
#define VICDefVectAddr (*((volatile uint32_t *) 0xFFFFF034))
#define VICVectAddr0   (*((volatile uint32_t *) 0xFFFFF100))
#define VICVectAddr1   (*((volatile uint32_t *) 0xFFFFF104))
#define VICVectAddr2   (*((volatile uint32_t *) 0xFFFFF108))
#define VICVectAddr3   (*((volatile uint32_t *) 0xFFFFF10C))
#define VICVectAddr4   (*((volatile uint32_t *) 0xFFFFF110))
#define VICVectAddr5   (*((volatile uint32_t *) 0xFFFFF114))
#define VICVectAddr6   (*((volatile uint32_t *) 0xFFFFF118))
#define VICVectAddr7   (*((volatile uint32_t *) 0xFFFFF11C))
#define VICVectAddr8   (*((volatile uint32_t *) 0xFFFFF120))
#define VICVectAddr9   (*((volatile uint32_t *) 0xFFFFF124))
#define VICVectAddr10  (*((volatile uint32_t *) 0xFFFFF128))
#define VICVectAddr11  (*((volatile uint32_t *) 0xFFFFF12C))
#define VICVectAddr12  (*((volatile uint32_t *) 0xFFFFF130))
#define VICVectAddr13  (*((volatile uint32_t *) 0xFFFFF134))
#define VICVectAddr14  (*((volatile uint32_t *) 0xFFFFF138))
#define VICVectAddr15  (*((volatile uint32_t *) 0xFFFFF13C))

#define VICVectCntl0     (*((volatile uint32_t *)   0xFFFFF200))
#define VICVectCntl0_bit (*(struct VICVecCntl0_reg*)0xFFFFF200)
struct VICVecCntl0_reg {
  uint32_t IRSIA: 5;
  uint32_t IRQslot_en: 1;
  uint32_t _: 26;
};

#define VICVectCntl1   (*((volatile uint32_t *) 0xFFFFF204))
#define VICVectCntl2   (*((volatile uint32_t *) 0xFFFFF208))
#define VICVectCntl3   (*((volatile uint32_t *) 0xFFFFF20C))
#define VICVectCntl4   (*((volatile uint32_t *) 0xFFFFF210))
#define VICVectCntl5   (*((volatile uint32_t *) 0xFFFFF214))
#define VICVectCntl6   (*((volatile uint32_t *) 0xFFFFF218))
#define VICVectCntl7   (*((volatile uint32_t *) 0xFFFFF21C))
#define VICVectCntl8   (*((volatile uint32_t *) 0xFFFFF220))
#define VICVectCntl9   (*((volatile uint32_t *) 0xFFFFF224))
#define VICVectCntl10  (*((volatile uint32_t *) 0xFFFFF228))
#define VICVectCntl11  (*((volatile uint32_t *) 0xFFFFF22C))
#define VICVectCntl12  (*((volatile uint32_t *) 0xFFFFF230))
#define VICVectCntl13  (*((volatile uint32_t *) 0xFFFFF234))
#define VICVectCntl14  (*((volatile uint32_t *) 0xFFFFF238))
#define VICVectCntl15  (*((volatile uint32_t *) 0xFFFFF23C))

#define VIC_WDT         0
#define VIC_ARMCore0    2
#define VIC_ARMCore1    3
#define VIC_TIMER0      4
#define VIC_TIMER1      5
#define VIC_UART0       6
#define VIC_UART1       7
#define VIC_PWM0        8
#define VIC_I2C0        9
#define VIC_SPI0        10
#define VIC_SSP         11
#define VIC_PLL         12
#define VIC_RTC         13
#define VIC_EINT0       14
#define VIC_EINT1       15
#define VIC_EINT2       16
#define VIC_EINT3       17
#define VIC_AD0         18
#define VIC_I2C1        19
#define VIC_BOD         20
#define VIC_AD1         21

#define VIC_NUM_INTS    22

#endif /* LPC2138_H */
