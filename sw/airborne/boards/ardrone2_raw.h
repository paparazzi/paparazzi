#ifndef CONFIG_ARDRONE2_RAW
#define CONFIG_ARDRONE2_RAW

#define BOARD_ARDRONE2_RAW

/* Default actuators driver */
#define DEFAULT_ACTUATORS "arch/omap/subsystems/actuators/actuators_ardrone2_raw.h"
#define ActuatorDefaultSet(_x,_y) ActuatorArdroneSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsArdroneInit()
#define ActuatorsDefaultCommit() ActuatorsArdroneCommit()

#endif /* CONFIG_ARDRONE2_RAW */
