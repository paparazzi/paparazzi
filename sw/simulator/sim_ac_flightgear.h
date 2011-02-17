#ifndef SIM_AC_FLIGHTGEAR_H
#define SIM_AC_FLIGHTGEAR_H


void sim_ac_flightgear_init(const char* host,  unsigned int port);
void sim_ac_flightgear_send(JSBSim::FGFDMExec* FDMExec);

#endif /* SIM_AC_FLIGHTGEAR_H */
