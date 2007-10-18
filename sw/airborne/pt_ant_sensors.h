#ifndef PT_ANT_SENSORS_H
#define PT_ANT_SENSORS_H

#include "std.h"

#if 0
struct PtAntSensorData {
  uint16_t accel_x;
  uint16_t accel_y;
  uint16_t accel_z;
  uint16_t mag_x;
  uint16_t mag_y;
  uint16_t mag_z;
  uint16_t checksum;
};
#else
struct PtAntSensorData {
  uint8_t accel_x;
  uint8_t accel_y;
  uint8_t accel_z;
};
#endif

extern volatile uint8_t pt_ant_sensors_data_available;
extern struct PtAntSensorData pt_ant_sensors_data;
extern void pt_ant_sensors_init(void);
extern void pt_ant_sensors_read(void);
extern void pt_ant_sensors_init_spi(void);

#define PtAntSensorsEventCheckAndHandle() {				                                                     \
    if (pt_ant_sensors_data_available) {				                                                     \
      pt_ant_sensors_data_available = FALSE;				                                                     \
      DOWNLINK_SEND_ANTENNA_DEBUG(&pt_ant_sensors_data.accel_x, &pt_ant_sensors_data.accel_y, &pt_ant_sensors_data.accel_z); \
    }									                                                     \
  }

#define PtAntSensorsPeriodic() { \
    pt_ant_sensors_read();	 \
  }


#endif /* PT_ANT_SENSORS_H */
