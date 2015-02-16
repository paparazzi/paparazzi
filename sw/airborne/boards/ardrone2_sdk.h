#ifndef CONFIG_ARDRONE2_SDK
#define CONFIG_ARDRONE2_SDK

#define BOARD_ARDRONE2_SDK

#define UART1_DEV "/dev/ttyUSB0"

/* Internal communication */
#define ARDRONE_NAVDATA_PORT 5554
#define ARDRONE_AT_PORT 5556
#define ARDRONE_NAVDATA_BUFFER_SIZE 4096
#define ARDRONE_IP "192.168.1.1"

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/ardrone/actuators_at.h"
#define ActuatorDefaultSet(_x,_y) {}
#define ActuatorsDefaultInit() {}
#define ActuatorsDefaultCommit() {}

#define USE_BARO_BOARD 0

#endif /* CONFIG_ARDRONE2_SDK */
