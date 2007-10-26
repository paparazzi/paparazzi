#ifndef IMU_V3_HW_H
#define IMU_V3_HW_H

#include "max1167.h"

extern void imu_v3_hw_init(void);

#define SPI_SLAVE_NONE 0
#define SPI_SLAVE_MAX  1
extern uint8_t spi_cur_slave;

#define ImuEventCheckAndHandle(user_handler) {	\
    if (max1167_data_available) {		\
      max1167_data_available = FALSE;		\
      spi_cur_slave = SPI_SLAVE_NONE;		\
      ImuUpdateGyros();				\
      ImuUpdateAccels();			\
      user_handler();				\
    }						\
  }

#define ImuPeriodic() {				\
    if (spi_cur_slave != SPI_SLAVE_NONE) {	\
      DOWNLINK_SEND_AHRS_OVERRUN();		\
    }						\
    else {					\
      spi_cur_slave = SPI_SLAVE_MAX;		\
      max1167_read();				\
    }						\
  }

#endif /* IMU_V3_HW_H */
