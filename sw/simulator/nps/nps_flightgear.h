#ifndef NPS_FLIGHTGEAR_H
#define NPS_FLIGHTGEAR_H

extern void nps_flightgear_init(const char* host,  unsigned int port, unsigned int port_in, unsigned int time_offset);
extern void nps_flightgear_send(void);
extern void nps_flightgear_send_fdm(void);

#endif /* NPS_FLIGHTGEAR_H */
