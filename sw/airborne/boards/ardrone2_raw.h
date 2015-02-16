#ifndef CONFIG_ARDRONE2_RAW
#define CONFIG_ARDRONE2_RAW

#define BOARD_ARDRONE2_RAW

#define UART1_DEV "/dev/ttyUSB0"

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/ardrone/actuators_ardrone2_raw.h"
#define ActuatorDefaultSet(_x,_y) ActuatorArdroneSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsArdroneInit()
#define ActuatorsDefaultCommit() ActuatorsArdroneCommit()


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_ARDRONE2_RAW */
