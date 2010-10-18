/* 
C Datei für die Einbindung eines ArduIMU's
Autoren@ZHAW: 	schmiemi
		chaneren
*/


#include <stdbool.h>
#include "ArduIMU.h"
#include "main_fbw.h"
#include "i2c.h"

// test
#include "estimator.h"

// für das Senden von GPS-Daten an den ArduIMU
#include "gps.h"
int32_t GPS_Data[13];
static volatile bool_t gps_i2c_done;

					
// Adresse des I2C Slaves: 	0001 0110	letztes Bit ist für Read/Write
// einzugebende Adresse im ArduIMU ist 0000 1011 
//da ArduIMU das Read/Write Bit selber anfügt.
#define ArduIMU_SLAVE_ADDR 0x22

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "uart.h"
#include "messages.h"
#include "downlink.h"

static volatile bool_t ArduIMU_i2c_done;
static int16_t recievedData[NB_DATA];
float ArduIMU_data[NB_DATA];
int8_t messageNr;
int8_t imu_daten_angefordert;
int8_t gps_daten_abgespeichert;
int8_t gps_daten_versendet_msg1;
int8_t gps_daten_versendet_msg2;

float arduimu_roll_neutral;
float arduimu_pitch_neutral;
float pitch_of_throttle_gain;
float throttle_slew;

void ArduIMU_init( void ) {
  ArduIMU_i2c_done = TRUE;
  gps_i2c_done = TRUE;
  i2c0_buf[0] = 0;
  i2c0_buf[1] = 0;
  messageNr = 0;
  imu_daten_angefordert = FALSE;
  gps_daten_abgespeichert = FALSE;
  gps_daten_versendet_msg1 = FALSE;
  gps_daten_versendet_msg2 = FALSE;
  arduimu_roll_neutral = ARDUIMU_ROLL_NEUTRAL;
  arduimu_pitch_neutral = ARDUIMU_PITCH_NEUTRAL;
  pitch_of_throttle_gain = PITCH_OF_THROTTLE_GAIN;
  throttle_slew = V_CTL_THROTTLE_SLEW;
}


void ArduIMU_periodicGPS( void ) {

	if ( gps_daten_versendet_msg1 == TRUE && gps_daten_versendet_msg2 == TRUE ) {
		gps_daten_abgespeichert = FALSE;
	}

	if( imu_daten_angefordert == TRUE ){
		IMU_Daten_verarbeiten();
	}

	if ( gps_daten_abgespeichert == FALSE ) {
		//posllh
		GPS_Data [0] = gps_itow;	
		GPS_Data [1] = gps_lon;		
		GPS_Data [2] = gps_lat;		
		GPS_Data [3] = gps_alt;			//höhe über elipsoid
		GPS_Data [4] = gps_hmsl;		//höhe über sea level
		//velend
		GPS_Data [5] = gps_speed;		//speed 3D
		GPS_Data [6] = gps_gspeed;		//ground speed	
		GPS_Data [7] = gps_course * 100000;	//Kurs
		//status
		GPS_Data [8] = gps_mode;		//fix
		GPS_Data [9] = gps_stauts_flag;		//flags
		//sol	
		GPS_Data [10] = gps_mode;		//fix
		GPS_Data [11] = gps_sol_flags;		//flags
		GPS_Data [12] = gps_ecefVZ;		//ecefVZ
		GPS_Data [13] = gps_numSV;
		
		gps_daten_abgespeichert = TRUE;
	}
	
	if(messageNr == 0) {

	  //test für 32bit in byte packete abzupacken:
	  //GPS_Data [0] = -1550138773;

	  i2c0_buf[0] = 0;				//message Nr = 0 --> itow bis ground speed
	  i2c0_buf[1] = (uint8_t) GPS_Data[0];		//itow
	  i2c0_buf[2] = (uint8_t) (GPS_Data[0] >>8);
	  i2c0_buf[3] = (uint8_t) (GPS_Data[0] >>16);
	  i2c0_buf[4] = (uint8_t) (GPS_Data[0] >>24);
	  i2c0_buf[5] = (uint8_t) GPS_Data[1];		//lon
	  i2c0_buf[6] = (uint8_t) (GPS_Data[1] >>8);
	  i2c0_buf[7] = (uint8_t) (GPS_Data[1] >>16);
	  i2c0_buf[8] = (uint8_t) (GPS_Data[1] >>24);
	  i2c0_buf[9] = (uint8_t) GPS_Data[2];		//lat
	  i2c0_buf[10] = (uint8_t) (GPS_Data[2] >>8);
	  i2c0_buf[11] = (uint8_t) (GPS_Data[2] >>16);
	  i2c0_buf[12] = (uint8_t) (GPS_Data[2] >>24);
	  i2c0_buf[13] = (uint8_t) GPS_Data[3];		//height
	  i2c0_buf[14] = (uint8_t) (GPS_Data[3] >>8);
	  i2c0_buf[15] = (uint8_t) (GPS_Data[3] >>16);
	  i2c0_buf[16] = (uint8_t) (GPS_Data[3] >>24);
	  i2c0_buf[17] = (uint8_t) GPS_Data[4];		//hmsl
	  i2c0_buf[18] = (uint8_t) (GPS_Data[4] >>8);
	  i2c0_buf[19] = (uint8_t) (GPS_Data[4] >>16);
	  i2c0_buf[20] = (uint8_t) (GPS_Data[4] >>24);
	  i2c0_buf[21] = (uint8_t) GPS_Data[5];		//speed
	  i2c0_buf[22] = (uint8_t) (GPS_Data[5] >>8);
	  i2c0_buf[23] = (uint8_t) (GPS_Data[5] >>16);
	  i2c0_buf[24] = (uint8_t) (GPS_Data[5] >>24);
	  i2c0_buf[25] = (uint8_t) GPS_Data[6];		//gspeed
	  i2c0_buf[26] = (uint8_t) (GPS_Data[6] >>8);
	  i2c0_buf[27] = (uint8_t) (GPS_Data[6] >>16);
	  i2c0_buf[28] = (uint8_t) (GPS_Data[6] >>24);
	  i2c0_transmit(ArduIMU_SLAVE_ADDR, 28, &gps_i2c_done);

	  gps_daten_versendet_msg1 = TRUE;
	  messageNr =1;
	}
        else {
		
	  i2c0_buf[0] = 1;			//message Nr = 1 --> ground course, ecefVZ, numSV, Fix, flags, fix, flags
	  i2c0_buf[1] = GPS_Data[7];		//ground course
  	  i2c0_buf[2] = (GPS_Data[7] >>8);
  	  i2c0_buf[3] = (GPS_Data[7] >>16);
	  i2c0_buf[4] = (GPS_Data[7] >>24);
	  i2c0_buf[5] = GPS_Data[12];		//ecefVZ
	  i2c0_buf[6] = (GPS_Data[12] >>8);
	  i2c0_buf[7] = (GPS_Data[12] >>16);
	  i2c0_buf[8] = (GPS_Data[12] >>24);
	  i2c0_buf[9] = GPS_Data[13];		//numSV
	  i2c0_buf[10] = GPS_Data[8];		//status gps fix
	  i2c0_buf[11] = GPS_Data[9];		//status flags
	  i2c0_buf[12] = GPS_Data[10];		//sol gps fix
	  i2c0_buf[13] = GPS_Data[11];		//sol flags
	  i2c0_transmit(ArduIMU_SLAVE_ADDR, 13, &gps_i2c_done);

	  gps_daten_versendet_msg2 = TRUE;	
	  messageNr = 0;
	}

        //DOWNLINK_SEND_DEBUG_ZHAW(DefaultChannel, &gps_mode , &gps_numSV ,&gps_alt , &gps_hmsl , &gps_itow, &gps_speed);
}

void ArduIMU_periodic( void ) {
//Frequenz des Aufrufs wird in conf/modules/ArduIMU.xml festgelegt.

	//I2C-Nachricht anfordern an Slave Adresse, erwartete Anzahl Byte, Status
	if (imu_daten_angefordert == TRUE) {	
		IMU_Daten_verarbeiten();
	}
	i2c0_receive(ArduIMU_SLAVE_ADDR, 12, &ArduIMU_i2c_done);
	imu_daten_angefordert = TRUE;
	/* 
	Buffer O:	Roll
	Buffer 1:	Pitch
	Buffer 2:	Yaw
	Buffer 3:	Beschleunigung X-Achse
	Buffer 4:	Beschleunigung Y-Achse
	Buffer 5:	Beschleunigung Z-Achse
	*/


	//Nachricht zum GCS senden
	// DOWNLINK_SEND_ArduIMU(DefaultChannel, &ArduIMU_data[0], &ArduIMU_data[1], &ArduIMU_data[2], &ArduIMU_data[3], &ArduIMU_data[4], &ArduIMU_data[5]);

    //    DOWNLINK_SEND_DEBUG_ZHAW(DefaultChannel, &airspeed_mode , &altitude_mode ,&amsys_baro, &amsys_baro, &amsys_airspeed_scaliert, &amsys_baro_scaliert);
}

void IMU_Daten_verarbeiten( void ) {
	//Empfangene Byts zusammenfügen und bereitstellen
	recievedData[0] = (i2c0_buf[1]<<8) | i2c0_buf[0];
	recievedData[1] = (i2c0_buf[3]<<8) | i2c0_buf[2];
	recievedData[2] = (i2c0_buf[5]<<8) | i2c0_buf[4];
	recievedData[3] = (i2c0_buf[7]<<8) | i2c0_buf[6];
	recievedData[4] = (i2c0_buf[9]<<8) | i2c0_buf[8];
	recievedData[5] = (i2c0_buf[11]<<8) | i2c0_buf[10];

	//Floats zurück transformieren. Transformation ist auf ArduIMU programmiert.
	ArduIMU_data[0] = (float) (recievedData[0] / (float)100);
	ArduIMU_data[1] = (float) (recievedData[1] / (float)100);
	ArduIMU_data[2] = (float) (recievedData[2] / (float)100);
	ArduIMU_data[3] = (float) (recievedData[3] / (float)100);
	ArduIMU_data[4] = (float) (recievedData[4] / (float)100);
	ArduIMU_data[5] = (float) (recievedData[5] / (float)100);

	// test
	estimator_phi  = (float)ArduIMU_data[0]*0.01745329252 - arduimu_roll_neutral;  
	estimator_theta  = (float)ArduIMU_data[1]*0.01745329252 - arduimu_pitch_neutral;
	imu_daten_angefordert = FALSE;
}


