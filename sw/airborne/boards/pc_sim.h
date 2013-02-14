#ifndef CONFIG_PC_SIM_H
#define CONFIG_PC_SIM_H


/* Default actuators driver */
#define DEFAULT_ACTUATORS "subsystems/actuators/actuators_4017.h"
#define ActuatorDefaultSet(_x,_y) Actuator4017Set(_x,_y)
#define ActuatorsDefaultInit() Actuators4017Init()
#define ActuatorsDefaultCommit() Actuators4017Commit()


#define DefaultVoltageOfAdc(adc) (1.0*adc)


#define BOARD_HAS_BARO 1

#endif /* CONFIG_PC_SIM_H */
