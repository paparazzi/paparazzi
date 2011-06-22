#ifndef ins_ardimu_h
#define ins_ardimu_h


extern float ins_roll_neutral;
extern float ins_pitch_neutral;

extern int renorm_sqrt_count;
extern int imu_overrun;
extern float imu_health;

void ins_ardu_init( void );
void ins_ardu_periodic( void );
void ins_ardu_event( void );

void ins_ardu_send_gps( void );
void ins_data_process( void );

#define RadOfADC(_adc, scale) RadOfDeg((_adc * scale))

#endif // ins_ardimu
