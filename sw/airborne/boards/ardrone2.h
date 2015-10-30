#ifndef CONFIG_ARDRONE2
#define CONFIG_ARDRONE2

#define BOARD_ARDRONE2

#include "peripherals/video_device.h"

#ifndef UART1_DEV
#define UART1_DEV /dev/ttyUSB0
#endif

#ifndef UARTSPLIT2_DEV
#define UARTSPLIT2_DEV /dev/ttyUSB1
#endif


/* Cameras */
extern struct video_config_t bottom_camera;
extern struct video_config_t front_camera;

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/ardrone/actuators.h"
#define ActuatorDefaultSet(_x,_y) ActuatorArdroneSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsArdroneInit()
#define ActuatorsDefaultCommit() ActuatorsArdroneCommit()


/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

#endif /* CONFIG_ARDRONE2 */
