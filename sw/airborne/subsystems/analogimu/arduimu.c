#include <math.h>

#include "wiring.h"
#include "vector.h"
#include "matrix.h"
#include "arduimu.h"

#include "dcm.h"

// Released under Creative Commons License
// Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
// Version 1.0 for flat board updated by Doug Weibel and Jose Julio
// Version 1.7 includes support for SCP1000 absolute pressure sensor

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

// olri #include <avr/eeprom.h>
// olri #include <Wire.h>

//**********************************************************************
//  This section contains USER PARAMETERS !!!
//
//**********************************************************************

// *** NOTE!   Hardware version - Can be used for v1 (daughterboards) or v2 (flat)
#define BOARD_VERSION 2 // 1 For V1 and 2 for V2

// Ublox gps is recommended!
#define GPS_PROTOCOL 1    // 1 - Ublox,  2 - EM406,  3 - NMEA    We have only tested with Ublox

// Enable Air Start uses Remove Before Fly flag - connection to pin 6 on ArduPilot
#define ENABLE_AIR_START 1  //  1 if using airstart/groundstart signaling, 0 if not
#define GROUNDSTART_PIN 8    //  Pin number used for ground start signal (recommend 10 on v1 and 8 on v2 hardware)

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation, 0=do not use speed filter

/*For debugging propurses*/
#define PRINT_DEBUG 0   //Will print Debug messages

//OUTPUTMODE=1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 will print accelerometer only data
#define OUTPUTMODE 1

#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_GPS 1     //Will print GPS data

// *** NOTE!   To use ArduIMU with ArduPilot you must select binary output messages
#define PRINT_BINARY 1   //Will print binary message and suppress ASCII messages (above)

// *** NOTE!   Performance reporting is only supported for Ublox.  Set to 0 for others
#define PERFORMANCE_REPORTING 1  //Will include performance reports in the binary output ~ 1/2 min

/* Support for optional magnetometer (1 enabled, 0 dissabled) */
#define USE_MAGNETOMETER 0 // use 1 if you want to make yaw gyro drift corrections using the optional magnetometer

/* Support for optional barometer (1 enabled, 0 dissabled) */
#define USE_BAROMETER 0 	// use 1 if you want to get altitude using the optional absolute pressure sensor
#define ALT_MIX	50			// For binary messages: GPS or barometric altitude.  0 to 100 = % of barometric.  For example 75 gives 25% GPS alt and 75% baro

//**********************************************************************
//  End of user parameters
//**********************************************************************

#define SOFTWARE_VER "1.7"

// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
//#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer
//#define GRAVITY 9.81

// olri #define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

// olri #define ToRad(x) (x*0.01745329252)  // *pi/180
// olri #define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/º/s, 3.22mV/ADC step => 1.03
// Tested values : 0.96,0.96,0.94
   // #define Gyro_Gain_X 0.92 //X axis Gyro gain
   // #define Gyro_Gain_Y 0.92 //Y axis Gyro gain
   // #define Gyro_Gain_Z 0.94 //Z axis Gyro gain
   // olri #define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
   // olri #define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
   // olri #define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

//olri #define Kp_ROLLPITCH 0.015
// olri #define Ki_ROLLPITCH 0.000010
// olri #define Kp_YAW 1.2
//#define Kp_YAW 2.5      //High yaw drift correction gain - use with caution!
// olri #define Ki_YAW 0.00005

/*UBLOX Maximum payload length*/
#define UBX_MAXPAYLOAD 56

#define ADC_WARM_CYCLES 75

//olri #define FALSE 0
// olri #define TRUE 1


//float G_Dt=0.02;    // Integration time (DCM algorithm)
//chni: changed to 50ms, according to HSB-Data
float G_Dt=0.05;

long timeNow=0; // Hold the milliseond value for now
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values
boolean groundstartDone = false;    // Used to not repeat ground start

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

//Magnetometer variables
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;

// Euler angles
float roll;
float pitch;
float yaw;

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=180;
float COGX=0; //Course overground X axis
float COGY=1; //Course overground Y axis

unsigned int cycleCount=0;
byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
};
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here


float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};

//GPS

//GPS stuff
union long_union {
	int32_t dword;
	uint8_t  byte[4];
} longUnion;

union int_union {
	int16_t word;
	uint8_t  byte[2];
} intUnion;

/*Flight GPS variables*/
int gpsFix=1; //This variable store the status of the GPS
int gpsFixnew=0; //used to flag when new gps data received - used for binary output message flags
int gps_fix_count = 5;		//used to count 5 good fixes at ground startup
long lat=0; // store the Latitude from the gps to pass to output
long lon=0; // Store the Longitude from the gps to pass to output
long alt_MSL=0; //This is the altitude in millimeters
long iTOW=0; //GPS Millisecond Time of Week
long alt=0;  //Height above Ellipsoid in millimeters
float speed_3d=0; //Speed (3-D)
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=90;//This is the runaway direction of you "plane" in degrees
float gc_offset = 0; // Force yaw output to ground course when fresh data available (only implemented for ublox&binary message)
byte numSV=0; //Number of Sats used.
float ecefVZ=0; //Vertical Speed in m/s
unsigned long GPS_timer=0;

#if GPS_PROTOCOL == 1
// GPS UBLOX
//byte ck_a=0;    // Packet checksum
//byte ck_b=0;
byte UBX_step=0;
byte UBX_class=0;
byte UBX_id=0;
byte UBX_payload_length_hi=0;
byte UBX_payload_length_lo=0;
byte UBX_payload_counter=0;
byte UBX_buffer[UBX_MAXPAYLOAD];
byte UBX_ck_a=0;
byte UBX_ck_b=0;
#endif

//ADC variables
volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile uint16_t analog_buffer[8];
volatile uint8_t analog_count[8];


 #if BOARD_VERSION == 1
  uint8_t sensors[6] = {0,2,1,3,5,4};   // Use these two lines for Hardware v1 (w/ daughterboards)
  int SENSOR_SIGN[]= {1,-1,1,-1,1,-1,-1,-1,-1};  //Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
 #endif

 #if BOARD_VERSION == 2
  uint8_t sensors[6] = {6,7,3,0,1,2};  // For Hardware v2 flat
  int SENSOR_SIGN[] = {1,-1,-1,1,-1,1,-1,-1,-1};
 #endif

 // Performance Monitoring variables
 // Data collected and reported for ~1/2 minute intervals
 #if PERFORMANCE_REPORTING == 1
 int mainLoop_count = 0;              //Main loop cycles since last report
 int G_Dt_max = 0.0;                  //Max main loop cycle time in milliseconds
 byte gyro_sat_count = 0;
 byte adc_constraints = 0;
 byte renorm_sqrt_count = 0;
 byte renorm_blowup_count = 0;
 byte gps_payload_error_count = 0;
 byte gps_checksum_error_count = 0;
 byte gps_pos_fix_count = 0;
 byte gps_nav_fix_count = 0;
 byte gps_messages_sent = 0;
 long perf_mon_timer = 0;
 #endif
 unsigned int imu_health = 65012;

//**********************************************************************
//  This section contains SCP1000_D11 PARAMETERS !!!
//**********************************************************************
#if USE_BAROMETER == 1
#define SCP_MODE        (9)             // 9 = high speed mode, 10 = high resolution mode
#define PRESSURE_ADDR   (0x11U)          // IIC address of the SCP1000
// ************   #define START_ALTITUDE  (217U)           // default initial altitude in m above sea level

// When we have to manage data transfers via IIC directly we need to use the following addresses
// IIC address of the SCP1000 device forms the Top 7 bits of the address with the R/W bit as the LSB
#define READ_PRESSURE_ADDR    (PRESSURE_ADDR<<1 | 1)
#define WRITE_PRESSURE_ADDR   (PRESSURE_ADDR<<1)

// SCP1000 Register addresses
#define SNS_ADDR_POPERATION     (0x03U)  // OPERATON register
#define SNS_ADDR_PSTATUS        (0x07U)  // STATUS register
#define SNS_ADDR_PPRESSURE      (0x80U)  // DATARD16 Register (pressure)
#define SNS_ADDR_DATARD8	(0x7FU)  // DAYARD8 Register
#define SNS_ADDR_PTEMP		(0x81U)	 // TEMPOUT Register (temperature)

#ifndef TRUE
#define TRUE          (0x01)
#endif
#ifndef FALSE
#define FALSE         (0x00)
#endif

int temp_unfilt = 0;
int temperature = 0;
unsigned long press = 0;
unsigned long press_filt = 0;
unsigned long press_gnd = 0;
long ground_alt = 0;				// Ground altitude in millimeters
long press_alt = 0;					// Pressure altitude in millimeters

#endif
