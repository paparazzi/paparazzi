#include "wiring.h"
/* This file was automatically generated.  Do not edit! */
void debug_print(char string[]);
void startup_air(void);
void debug_handler(byte message);
void startup_ground(void);
//void loop();
#if USE_BAROMETER == 1
extern long press_alt;
extern long ground_alt;
extern unsigned long press_gnd;
extern unsigned long press_filt;
extern unsigned long press;
extern int temperature;
extern int temp_unfilt;
#endif
extern unsigned int imu_health;
extern long perf_mon_timer;
extern byte gps_messages_sent;
extern byte gps_nav_fix_count;
extern byte gps_pos_fix_count;
extern byte gps_checksum_error_count;
extern byte gps_payload_error_count;
extern byte renorm_blowup_count;
extern byte renorm_sqrt_count;
extern byte adc_constraints;
extern byte gyro_sat_count;
extern int G_Dt_max;
extern int SENSOR_SIGN[];
extern volatile uint8_t analog_count[8];
extern volatile uint16_t analog_buffer[8];
extern volatile uint8_t analog_reference;
extern volatile uint8_t MuxSel;
#if GPS_PROTOCOL == 1
extern byte UBX_ck_b;
extern byte UBX_ck_a;
extern byte UBX_buffer[UBX_MAXPAYLOAD];
extern byte UBX_payload_counter;
extern byte UBX_payload_length_lo;
extern byte UBX_payload_length_hi;
extern byte UBX_id;
extern byte UBX_class;
extern byte UBX_step;
extern byte ck_b;
extern byte ck_a;
#endif
extern unsigned long GPS_timer;
extern float ecefVZ;
extern byte numSV;
extern float gc_offset;
extern float ground_course;
extern float ground_speed;
extern float speed_3d;
extern long alt;
extern long iTOW;
extern long alt_MSL;
extern long lon;
extern long lat;
extern int gps_fix_count;
extern int gpsFixnew;
extern int gpsFix;
extern float Temporary_Matrix[3][3];
extern float Update_Matrix[3][3];
extern float DCM_Matrix[3][3];
extern byte gyro_sat;
extern unsigned int cycleCount;
extern float COGY;
extern float COGX;
extern float errorCourse;
extern float errorYaw[3];
extern float errorRollPitch[3];
extern float yaw;
extern float pitch;
extern float roll;
extern float MAG_Heading;
extern int magnetom_z;
extern int magnetom_y;
extern int magnetom_x;
extern float Omega[3];
extern float Omega_I[3];
extern float Omega_P[3];
extern float Omega_Vector[3];
extern float Gyro_Vector[3];
extern float Accel_Vector[3];
extern float AN_OFFSET[8];
extern float AN[8];
extern boolean groundstartDone;
extern long timer24;
extern long timer_old;
extern long timer;
extern long timeNow;
extern float G_Dt;
#define FALSE 0
#define TRUE 1
//#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer
//chni:
#define GRAVITY 9.9074 // typical 9.81 sensor depend, hier Z Sensor Wert unserer IMU in Normallage

//#define Kp_ROLLPITCH 0.2
#define Kp_ROLLPITCH 0.015
//#define Kp_YAW 1.2
#define Kp_YAW 1.2      //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005
#define Ki_ROLLPITCH 0.000010
#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation, 0=do not use speed filter
#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi
#define Accel_Scale(x) x*(GRAVITY/9.81) //Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define USE_DEFEKT
#ifdef USE_DEFEKT
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
#else
#define Gyro_Scaled_X(x) x*(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second
#endif

#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.94 //Z axis Gyro gain
//#define Gyro_Gain_X 1.0 //X axis Gyro gain
//#define Gyro_Gain_Y 1.0 //Y axis Gyro gain
//#define Gyro_Gain_Z 1.0 //Z axis Gyro gain

void dcm_init( int i );

extern float adc[11];
