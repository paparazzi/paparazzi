#ifndef LPC2138_H
#define LPC2138_H

#include <sys/types.h>

/* Phase Locked Loop (PLL) */
#define PLLCON         (*((volatile unsigned char*) 0xE01FC080))
#define PLLCON_bit     (*(struct PLLCON_reg*) PLLCON)
struct PLLCON_reg {
  unsigned char PLLE:1;
  unsigned char PLLC:1;
};

#define PLLCFG         (*((volatile unsigned char*) 0xE01FC084))
#define PLLCFG_bit     (*(struct PLLCFG_reg*) PLLCFG)
struct PLLCFG_reg {
  unsigned char MSEL:5; 
  unsigned char PSEL:2;
};

#define PLLSTAT        (*((volatile unsigned short*) 0xE01FC088))
#define PLLSTAT_bit    (*(struct PLLSTAT_reg*) PLLSTAT)
struct PLLSTAT_reg {
  unsigned short MSEL:5; 
  unsigned short PSEL:2;
  unsigned short _:1;
  unsigned short PLLE:1;
  unsigned short PLLC:1;
  unsigned short PLOCK:1;
};

#define PLLFEED        (*((volatile unsigned char*) 0xE01FC08C))
#define PLLFEED_bit    (*(struct PLLFEED_reg*) PLLFEED)
struct PLLFEED_reg {
  unsigned char FEED:8; 
};

/* Universal Asynchronous Receiver Transmitter 0 (UART0) */
#define U0LCR          (*((volatile unsigned char *) 0xE000C00C))
#define U0LCR_bit      (*(struct U0LCR_reg*) U0LCR)
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
