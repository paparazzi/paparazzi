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
  unsigned char PLLE:1;
  unsigned char PLLC:1;
};

#define PLLCFG         (*((volatile uint8_t*) 0xE01FC084))
#define PLLCFG_bit     (*(struct PLLCFG_reg*) 0xE01FC084)
struct PLLCFG_reg {
  unsigned char MSEL:5; 
  unsigned char PSEL:2;
};

#define PLLSTAT        (*((volatile uint16_t*) 0xE01FC088))
#define PLLSTAT_bit    (*(struct PLLSTAT_reg*) 0xE01FC088)
struct PLLSTAT_reg {
  unsigned short MSEL:5; 
  unsigned short PSEL:2;
  unsigned short _:1;
  unsigned short PLLE:1;
  unsigned short PLLC:1;
  unsigned short PLOCK:1;
};

#define PLLFEED        (*((volatile uint8_t*) 0xE01FC08C))
#define PLLFEED_bit    (*(struct PLLFEED_reg*) 0xE01FC08C)
struct PLLFEED_reg {
  unsigned char FEED:8; 
};


/* Timer 0 */
#define T0IR           (*((volatile uint32_t *) 0xE0004000))
#define T0TCR          (*((volatile uint32_t *) 0xE0004004))
#define T0TC           (*((volatile uint32_t *) 0xE0004008))
#define T0PR           (*((volatile uint32_t *) 0xE000400C))
#define T0PC           (*((volatile uint32_t *) 0xE0004010))
#define T0MCR          (*((volatile uint32_t *) 0xE0004014))
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



/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define U0LCR          (*((volatile uint8_t *) 0xE000C00C))
#define U0LCR_bit      (*(struct U0LCR_reg*) 0xE000C00C)
struct U0LCR_reg {
  unsigned char WLS:2;
  unsigned char SBS:1;
  unsigned char PE:1;
  unsigned char PS:2;
  unsigned char BC:1;
  unsigned char DLAB:1;
};
#define U0DLL          (*((volatile unsigned char *) 0xE000C000))
#define U0DLM          (*((volatile unsigned char *) 0xE000C004))

#endif /* LPC2138_H */
