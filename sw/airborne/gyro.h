#ifndef GYRO_H
#define GYRO_H

extern int16_t roll_rate_adc;
extern float roll_rate;

#if defined SPARK_FUN
extern float temp_comp;
#elif defined IDC300
extern float pitch_rate;
#endif

void gyro_init( void );
void gyro_update( void );


#endif /* GYRO_H */
