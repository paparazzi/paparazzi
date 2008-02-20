#ifndef SCP1000_H
#define SCP1000_H

#include "std.h"

extern void scp1000_init(void);

#define SCP1000_STA_STOPPED         0
#define SCP1000_STA_WAIT_EOC        1
#define SCP1000_STA_GOT_EOC         2
#define SCP1000_STA_SENDING_REQUEST 3
#define SCP1000_STA_DATA_AVAILABLE  4

extern volatile uint8_t  scp1000_status;
extern volatile uint32_t scp1000_pressure;

extern void scp1000_hw_init(void);
#include "scp1000_hw.h"

#endif /* SCP1000_H */
