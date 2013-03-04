#ifndef LOGGER_SPILINK_H_
#define LOGGER_SPILINK_H_

#include "std.h"
#define USE_DMA1_C3_IRQ
extern void logger_spiLink_init(void);
extern void logger_spiLink_periodic(void);


#define PACKED __attribute__((__packed__))

struct PACKED LoggerData {
    int32_t gyro_p;
    int32_t gyro_q;
    int32_t gyro_r;
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
};

#endif /* LOGGER_SPILINK_H_ */
