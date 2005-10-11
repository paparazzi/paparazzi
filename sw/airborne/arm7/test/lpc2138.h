#ifndef LPC2138_H
#define LPC2138_H

#include <sys/types.h>

#define PLLCON         (*((volatile __uint32_t*) 0xE01FC080))
#define PLLCON_bit     (*(struct PLLCON_reg*) PLLCON)
struct PLLCON_reg {
  __uint32_t PLLE:1;
  __uint32_t PLLC:1;
};

#define PLLCFG         (*((volatile __uint32_t*) 0xE01FC084))
#define PLLCFG_bit     (*(struct PLLCFG_reg*) PLLCFG)
struct PLLCFG_reg {
  __uint32_t MSEL:5; 
  __uint32_t PSEL:2;
};

#define PLLSTAT        (*((volatile __uint32_t*) 0xE01FC088))
#define PLLSTAT_bit    (*(struct PLLSTAT_reg*) PLLSTAT)
struct PLLSTAT_reg {
  __uint32_t MSEL:5; 
  __uint32_t PSEL:2;
  __uint32_t _:1;
  __uint32_t PLLE:1;
  __uint32_t PLLC:1;
  __uint32_t PLOCK:1;
};

#define PLLFEED        (*((volatile __uint32_t*) 0xE01FC08C))
#define PLLFEED_bit    (*(struct PLLFEED_reg*) PLLFEED)
struct PLLFEED_reg {
  __uint32_t FEED:8; 
};

#endif /* LPC2138_H */
