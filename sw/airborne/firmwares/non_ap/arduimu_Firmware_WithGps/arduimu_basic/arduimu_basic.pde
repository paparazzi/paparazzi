// Released under Creative Commons License 
// Code by Jordi Munoz and William Premerlani, Supported by Chris Anderson (Wired) and Nathan Sindle (SparkFun).
// Version 1.0 for flat board updated by Doug Weibel and Jose Julio
// Version 1.7 includes support for SCP1000 absolute pressure sensor
// Version modified by Gautier Hattenberger for use with Paparazzi autopilot only (no barometer, no compass)

// Axis definition: X axis pointing forward, Y axis pointing to the right and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise

#include <avr/eeprom.h>
#include <Wire.h>

//**********************************************************************
//  This section contains USER PARAMETERS !!!
//
//**********************************************************************

// *** NOTE!   Hardware version - Can be used for v1 (daughterboards) or v2 (flat)
#define BOARD_VERSION 2 // 1 For V1 and 2 for V2

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 2 // >1 use min speed filter for yaw drift cancellation, 0=do not use speed filter
// SPEEDFILT is also used to prevent ground_start calibration when moving

/*For debugging propurses*/
#define PRINT_DEBUG 0   //Will print Debug messages

#define PRINT_I2C_MSG 0 //Will print the I2C output buffer
#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data
#define PRINT_EULER 0   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_GPS 0     //Will print GPS data




//**********I2C Parameter ********************************************
int I2C_Message_ar[9];

//********************************************************************


// *** NOTE!   To use ArduIMU with ArduPilot you must select binary output messages (change to 1 here)
#define PRINT_BINARY 1   //Will print binary message and suppress ASCII messages (above)

// *** NOTE!   Performance reporting is only supported for Ublox.  Set to 0 for others
#define PERFORMANCE_REPORTING 1  //Will include performance reports in the binary output ~ 1/2 min

//**********************************************************************
//  End of user parameters
//**********************************************************************

#define SOFTWARE_VER "1.7"

// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/º/s, 3.22mV/ADC step => 1.03
#define Gyro_Gain_X 1.0 //X axis Gyro gain
#define Gyro_Gain_Y 1.0 //Y axis Gyro gain
#define Gyro_Gain_Z 1.0 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.015
#define Ki_ROLLPITCH 0.000010
#define Kp_YAW 1.2
//#define Kp_YAW 2.5      //High yaw drift correction gain - use with caution!
#define Ki_YAW 0.00005

#define ADC_WARM_CYCLES 100

#define FALSE 0
#define TRUE 1


float G_Dt=0.02;    // Integration time (DCM algorithm)

long timeNow=0; // Hold the milliseond value for now
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 

float AN[8]; //array that store the 6 ADC filtered data
float AN_OFFSET[8]; //Array that stores the Offset of the gyros

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

boolean high_accel_flag = false; // disable update on accelerometers if true
boolean calibrate_neutrals = false; // perform a calibration of gyro and accel neutrals

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

float DCM_Matrix[3][3] = {
  { 1,0,0 },
  { 0,1,0 },
  { 0,0,1 }
};

float Update_Matrix[3][3] = {
  { 0,1,2 },
  { 3,4,5 },
  { 6,7,8 }
}; //Gyros here

float Temporary_Matrix[3][3]={
  { 0,0,0 },
  { 0,0,0 },
  { 0,0,0 }
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
float speed_3d=0; //Speed (3-D)
float ground_speed=0;// This is the velocity your "plane" is traveling in meters for second, 1Meters/Second= 3.6Km/H = 1.944 knots
float ground_course=0;//This is the runaway direction of you "plane" in radians
unsigned long GPS_timer=0;

//***********************GPS PAPARAZZI************************************************************************
#define PPRZ_MAXPAYLOAD 32
byte Paparazzi_GPS_buffer[PPRZ_MAXPAYLOAD];
byte stGpsFix;
//************************************************************************************************************

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

//******************************************************************
void setup()
{ 
  Serial.begin(38400);
  pinMode(2,OUTPUT); //Serial Mux
  digitalWrite(2,HIGH); //Serial Mux
  pinMode(5,OUTPUT); //Red LED
  pinMode(6,OUTPUT); // Blue LED
  pinMode(7,OUTPUT); // Yellow LED

  //************Define I2C Output Handler************************
  Wire.begin(17);                // join i2c bus with address #2
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent); // register event --> Output
  //*************************************************************

  Analog_Reference(EXTERNAL); //Using external analog reference
  Analog_Init();

  debug_print("Welcome...");

#if BOARD_VERSION == 1
  debug_print("You are using Hardware Version 1...");
#endif 

#if BOARD_VERSION == 2
  debug_print("You are using Hardware Version 2...");
#endif 

  //Serial.print("ArduIMU: v");
  //Serial.println(SOFTWARE_VER);
  debug_handler(0);//Printing version

  startup_air();
  debug_handler(1);

  delay(250);
  Read_adc_raw();     // ADC initialization
  timer=DIYmillis();
  delay(20);

}

//***************************************************************************************
void loop() //Main Loop
{
  timeNow = millis();

  // 20 -> Main loop runs at 50Hz
  // 5 -> Main loop runs at 200Hz
  // Max measured duty time around 3ms
  if((timeNow-timer)>=5)
  {
    timer_old = timer;
    timer = timeNow;

#if PERFORMANCE_REPORTING == 1
    mainLoop_count++;
    if (timer-timer_old > G_Dt_max) G_Dt_max = timer-timer_old;
#endif

    G_Dt = (timer-timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    if(G_Dt > 1) G_Dt = 0;  //Something is wrong - keeps dt from blowing up, goes to zero to keep gyros from departing

    // *** DCM algorithm

    Read_adc_raw();

    Matrix_update(); 

    Normalize();

    Drift_correction();

    Euler_angles();

#if PRINT_BINARY == 1
    printdata(); //Send info via serial
#endif

    //Turn on the LED when you saturate any of the gyros.
    if((abs(Gyro_Vector[0])>=ToRad(300))||(abs(Gyro_Vector[1])>=ToRad(300))||(abs(Gyro_Vector[2])>=ToRad(300)))
    {
      gyro_sat=1;
#if PERFORMANCE_REPORTING == 1
      gyro_sat_count++;
#endif
      digitalWrite(5,HIGH);  
    }

    cycleCount++;

    // Do these things every 6th time through the main cycle 
    // This section gets called every 1000/(20*6) = 8 1/3 Hz
    // doing it this way removes the need for another 'millis()' call
    // and balances the processing load across main loop cycles.
    switch (cycleCount) {
      case(0):
        if(DIYmillis() - GPS_timer > 2000) {
          digitalWrite(6, LOW);  //If we don't receive any byte in two seconds turn off gps fix LED...
          debug_print("No GPS data"); 
          gpsFix = 1;
        } 
        break;

      case(1):
        //Here we will check if we are getting a signal to ground start and speed is high enough
        if (calibrate_neutrals && ground_speed < SPEEDFILT) {
          startup_ground();
          calibrate_neutrals = false;
        }
        break;

      case(2):
        // GYRO Saturation indication
        if(gyro_sat>=1) {
          digitalWrite(5,HIGH); //Turn Red LED when gyro is saturated. 
          if(gyro_sat>=8)  // keep the LED on for 8/10ths of a second
            gyro_sat=0;
          else
            gyro_sat++;
        } else {
          digitalWrite(5,LOW);
        }
        break;

      case(3):
        // YAW drift correction indication
        if(ground_speed<SPEEDFILT) {
          digitalWrite(7,HIGH);    //  Turn on yellow LED if speed too slow and yaw correction supressed
        } else {
          digitalWrite(7,LOW);
        }
        break;

      case(4):
        // GPS Fix indication
        if(gpsFix==0) {  // yep its backwards 0 means a good fix in GPS world!
          digitalWrite(6,HIGH);  //Turn Blue LED when gps is fixed. 
        } else {
          digitalWrite(6,LOW);
        }
        break;

      case(5):
        cycleCount = -1;		// Reset case counter, will be incremented to zero before switch statement
        break;
    }


#if PERFORMANCE_REPORTING == 1
    if (timeNow-perf_mon_timer > 20000) {
      printPerfData(timeNow-perf_mon_timer);
      perf_mon_timer=timeNow;
    }
#endif

  }

}

//********************************************************************************
void startup_ground(void)
{
  uint16_t store=0;
  int flashcount = 0;

  debug_handler(2);
  for(int c=0; c<ADC_WARM_CYCLES; c++)
  { 
    digitalWrite(7,LOW);
    digitalWrite(6,HIGH);
    digitalWrite(5,LOW);
    delay(50);
    Read_adc_raw();
    digitalWrite(7,HIGH);
    digitalWrite(6,LOW);
    digitalWrite(5,HIGH);
    delay(50);
  }

  Read_adc_raw();
  delay(20);
  Read_adc_raw();
  for(int y=0; y<=5; y++)   // Read first initial ADC values for offset.
    AN_OFFSET[y]=AN[y];

  for(int i=0;i<400;i++)    // We take some readings...
  {
    Read_adc_raw();
    for(int y=0; y<=5; y++)   // Read initial ADC values for offset (averaging).
      AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
    delay(20);
    if(flashcount == 5) {
      digitalWrite(7,LOW);
      digitalWrite(6,HIGH);
      digitalWrite(5,LOW);
    }
    if(flashcount >= 10) {
      flashcount = 0;
      digitalWrite(7,HIGH);
      digitalWrite(6,LOW);
      digitalWrite(5,HIGH);
    }
    flashcount++;
  }
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);

  AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

  // Store parameters in eeprom
  for(int y=0; y<=5; y++)
  {
    Serial.println(AN_OFFSET[y]);
    store = ((AN_OFFSET[y]-200.f)*100.0f);
    eeprom_busy_wait();
    eeprom_write_word((uint16_t *)	(y*2+2), store);	
  }

  debug_handler(6);
}

//************************************************************************************
void startup_air(void)
{
  uint16_t temp=0;

  for(int y=0; y<=5; y++)
  {
    eeprom_busy_wait();
    temp = eeprom_read_word((uint16_t *)	(y*2+2));
    AN_OFFSET[y] = temp/100.f+200.f;	
    Serial.println(AN_OFFSET[y]);
  }
  debug_handler(5);
}    


void debug_print(char string[])
{
#if PRINT_DEBUG != 0 
  Serial.print("???");
  Serial.print(string);
  Serial.println("***");
#endif
}

void debug_handler(byte message)
{
#if PRINT_DEBUG != 0 

  static unsigned long BAD_Checksum=0;

  switch(message) 
  {
    case 0:
      Serial.print("???Software Version ");
      Serial.print(SOFTWARE_VER);
      Serial.println("***");
      break;

    case 1:
      Serial.println("???Air Start!***");
      break;

    case 2:
      Serial.println("???Ground Start!***");
      break;      

    case 3:
      Serial.println("???Enabling Magneto...***");
      break;  

    case 4:
      Serial.println("???Enabling Pressure Altitude...***");
      break;         

    case 5:
      Serial.println("???Air Start complete");	
      break;     

    case 6:
      Serial.println("???Ground Start complete");	
      break;

    case 10:
      BAD_Checksum++;
      Serial.print("???GPS Bad Checksum: "); 
      Serial.print(BAD_Checksum);
      Serial.println("...***");
      break;

    default:
      Serial.println("???Invalid debug ID...***");
      break;

  }
#endif

}

/*
EEPROM memory map

0 0x00		Unused
1 0x01 		..
2 0x02 		AN_OFFSET[0]
3 0x03 		..
4 0x04 		AN_OFFSET[1]
5 0x05 		..
6 0x06 		AN_OFFSET[2]
7 0x07 		..
8 0x08 		AN_OFFSET[3]
9 0x09 		..
10 0x0A		AN_OFFSET[4]
11 0x0B		..
12 0x0C		AN_OFFSET[5]
13 0x0D		..	
14 0x0E		Unused
15 0x0F		..
16 0x10		Ground Pressure
17 0x11		..
18 0x12		..
19 0x13		..
20 0x14		Ground Temp
21 0x15		..
22 0x16		Ground Altitude
23 0x17		..
*/
