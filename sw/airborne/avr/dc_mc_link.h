#ifndef DC_MC_LINK_H
#define DC_MC_LINK_H

#include "std.h"

extern volatile uint8_t dc_mc_link_event;
extern uint16_t dc_mc_link_command;
extern uint8_t dc_mc_link_twi_rx_buf[];

extern void dc_mc_link_init(void);
extern void dc_mc_link_periodic(void);


#endif /* DC_MC_LINK_H */
