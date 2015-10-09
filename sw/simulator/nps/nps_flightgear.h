#ifndef NPS_FLIGHTGEAR_H
#define NPS_FLIGHTGEAR_H


extern void nps_flightgear_init(const char *host,  unsigned int port, unsigned int time_offset);
extern void nps_flightgear_send();
extern void nps_flightgear_send_fdm();

#endif /* NPS_FLIGHTGEAR_H */
