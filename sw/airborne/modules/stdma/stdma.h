#ifndef STDMA_H_
#define STDMA_H_

#include "stdint.h"
#include "subsystems/datalink/bluegiga.h"

extern struct pprz_transport stdma_trans;

extern void stdma_init(void);
extern void stdma_periodic(void);
extern void read_message(void);
extern void write_message(void);

extern void set_stdma_data(uint8_t *stdma_data, uint8_t data_len);

#endif  // STDMA_H_
