#ifndef MAX11040_H
#define MAX11040_H


#include "std.h"

#define MAXM_NB_CHAN 16
#define MAXM_NB_ADCS ((MAXM_NB_CHAN+3)/4)
#define MAX11040_BUF_SIZE 320

extern volatile uint8_t max11040_status;
extern volatile uint8_t max11040_data;
extern volatile int32_t max11040_values[MAX11040_BUF_SIZE][MAXM_NB_CHAN];
extern volatile uint32_t max11040_timestamp[MAX11040_BUF_SIZE];
extern volatile uint8_t max11040_count;
extern volatile uint32_t max11040_buf_in;
extern volatile uint32_t max11040_buf_out;


#define MAX11040_RESET      0
#define MAX11040_CONF       1
#define MAX11040_INSTANT    2
#define MAX11040_RATE       3
#define MAX11040_DONE       4
#define MAX11040_DATA       5
#define MAX11040_DATA2      6

#define MAX11040_IDLE            0
#define MAX11040_DATA_AVAILABLE  1

void max11040_init(void);
void max11040_periodic(void);


#endif /* MAX11040_H */

