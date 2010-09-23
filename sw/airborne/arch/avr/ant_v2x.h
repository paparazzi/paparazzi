#ifndef ANT_V2X_H
#define ANT_V2X_H

#include "std.h"

extern void ant_v2x_init( void );
extern void ant_v2x_periodic(void);
extern bool_t ant_v2x_is_in_calibration(void);
extern void ant_v2x_reset( void );
extern void ant_v2x_read_data( void );

struct Ant_V2xConfig
{
  float   declination;
  uint8_t true_north;
  uint8_t cal_sample_freq;
  uint8_t sample_freq;
  uint8_t period;
  uint8_t big_idian;
  uint8_t damping_size;
};

struct Ant_V2xData
{
  int32_t xraw;
  int32_t yraw;
  float xcal;
  float ycal;
  float heading;
  float magnitude;
  float temp;
  uint8_t distor;
  uint8_t cal_status;
};

struct Ant_V2xCal
{
  int8_t byte_count;
  int32_t x_offset;
  int32_t y_offset;
  int32_t x_gain;
  int32_t y_gain;
  float phi;
  float cal_magnitude;
};

extern volatile bool_t ant_v2x_data_available;
extern struct Ant_V2xData ant_v2x_data;

#endif /* ANT_V2X_H */

