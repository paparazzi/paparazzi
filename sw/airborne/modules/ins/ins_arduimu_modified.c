/*
C Datei für die Einbindung eines ArduIMU's
Autoren@ZHAW:   schmiemi
        chaneren
*/


#include <stdbool.h>
#include "modules/ins/ins_arduimu_modified.h"
#include "firmwares/fixedwing/main_fbw.h"
#include "mcu_periph/i2c.h"

// test
#include "estimator.h"

// für das Senden von GPS-Daten an den ArduIMU
#include "gps.h"
int32_t GPS_Data[14];

#ifndef ARDUIMU_I2C_DEV
#define ARDUIMU_I2C_DEV i2c0
#endif

// Adresse des I2C Slaves:  0001 0110	letztes Bit ist für Read/Write
// einzugebende Adresse im ArduIMU ist 0000 1011
//da ArduIMU das Read/Write Bit selber anfügt.
#define ArduIMU_SLAVE_ADDR 0x22

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

struct i2c_transaction ardu_gps_trans;
struct i2c_transaction ardu_ins_trans;

static int16_t recievedData[NB_DATA];
float ArduIMU_data[NB_DATA];

float ins_roll_neutral;
float ins_pitch_neutral;

void ArduIMU_init( void ) {
  ardu_ins_trans.status = I2CTransDone;
  ardu_gps_trans.status = I2CTransDone;

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
}


#define GPS_DATA_MSG1   0
#define GPS_DATA_MSG2   1

void ArduIMU_periodicGPS( void ) {
  static uint8_t gps_data_status = GPS_DATA_MSG1;

  if (ardu_gps_trans.status != I2CTransDone) { return; }

  if ( gps_data_status == GPS_DATA_MSG1 ) {
    //posllh
    GPS_Data [0] = gps_itow;
    GPS_Data [1] = gps_lon;
    GPS_Data [2] = gps_lat;
    GPS_Data [3] = gps_alt;			//höhe über elipsoid
    GPS_Data [4] = gps_hmsl;		//höhe über sea level
    //velned
    GPS_Data [5] = gps_speed_3d;		//speed 3D
    GPS_Data [6] = gps_gspeed;		//ground speed
    GPS_Data [7] = gps_course * 100000;	//Kurs
    //status
    GPS_Data [8] = gps_mode;		//fix
    GPS_Data [9] = gps_status_flags;	//flags
    //sol
    GPS_Data [10] = gps_mode;		//fix
    GPS_Data [11] = gps_sol_flags;		//flags
    GPS_Data [12] = gps_ecefVZ;		//ecefVZ
    GPS_Data [13] = gps_numSV;

    //test für 32bit in byte packete abzupacken:
    //GPS_Data [0] = -1550138773;

    ardu_gps_trans.buf[0] = 0;				//message Nr = 0 --> itow bis ground speed
    ardu_gps_trans.buf[1] = (uint8_t) GPS_Data[0];		//itow
    ardu_gps_trans.buf[2] = (uint8_t) (GPS_Data[0] >>8);
    ardu_gps_trans.buf[3] = (uint8_t) (GPS_Data[0] >>16);
    ardu_gps_trans.buf[4] = (uint8_t) (GPS_Data[0] >>24);
    ardu_gps_trans.buf[5] = (uint8_t) GPS_Data[1];		//lon
    ardu_gps_trans.buf[6] = (uint8_t) (GPS_Data[1] >>8);
    ardu_gps_trans.buf[7] = (uint8_t) (GPS_Data[1] >>16);
    ardu_gps_trans.buf[8] = (uint8_t) (GPS_Data[1] >>24);
    ardu_gps_trans.buf[9] = (uint8_t) GPS_Data[2];		//lat
    ardu_gps_trans.buf[10] = (uint8_t) (GPS_Data[2] >>8);
    ardu_gps_trans.buf[11] = (uint8_t) (GPS_Data[2] >>16);
    ardu_gps_trans.buf[12] = (uint8_t) (GPS_Data[2] >>24);
    ardu_gps_trans.buf[13] = (uint8_t) GPS_Data[3];		//height
    ardu_gps_trans.buf[14] = (uint8_t) (GPS_Data[3] >>8);
    ardu_gps_trans.buf[15] = (uint8_t) (GPS_Data[3] >>16);
    ardu_gps_trans.buf[16] = (uint8_t) (GPS_Data[3] >>24);
    ardu_gps_trans.buf[17] = (uint8_t) GPS_Data[4];		//hmsl
    ardu_gps_trans.buf[18] = (uint8_t) (GPS_Data[4] >>8);
    ardu_gps_trans.buf[19] = (uint8_t) (GPS_Data[4] >>16);
    ardu_gps_trans.buf[20] = (uint8_t) (GPS_Data[4] >>24);
    ardu_gps_trans.buf[21] = (uint8_t) GPS_Data[5];		//speed
    ardu_gps_trans.buf[22] = (uint8_t) (GPS_Data[5] >>8);
    ardu_gps_trans.buf[23] = (uint8_t) (GPS_Data[5] >>16);
    ardu_gps_trans.buf[24] = (uint8_t) (GPS_Data[5] >>24);
    ardu_gps_trans.buf[25] = (uint8_t) GPS_Data[6];		//gspeed
    ardu_gps_trans.buf[26] = (uint8_t) (GPS_Data[6] >>8);
    ardu_gps_trans.buf[27] = (uint8_t) (GPS_Data[6] >>16);
    ardu_gps_trans.buf[28] = (uint8_t) (GPS_Data[6] >>24);
    I2CTransmit(ARDUIMU_I2C_DEV, ardu_gps_trans, ArduIMU_SLAVE_ADDR, 28);

    gps_data_status = GPS_DATA_MSG2;
  }
  else {

    ardu_gps_trans.buf[0] = 1;			//message Nr = 1 --> ground course, ecefVZ, numSV, Fix, flags, fix, flags
    ardu_gps_trans.buf[1] = GPS_Data[7];		//ground course
    ardu_gps_trans.buf[2] = (GPS_Data[7] >>8);
    ardu_gps_trans.buf[3] = (GPS_Data[7] >>16);
    ardu_gps_trans.buf[4] = (GPS_Data[7] >>24);
    ardu_gps_trans.buf[5] = GPS_Data[12];		//ecefVZ
    ardu_gps_trans.buf[6] = (GPS_Data[12] >>8);
    ardu_gps_trans.buf[7] = (GPS_Data[12] >>16);
    ardu_gps_trans.buf[8] = (GPS_Data[12] >>24);
    ardu_gps_trans.buf[9] = GPS_Data[13];		//numSV
    ardu_gps_trans.buf[10] = GPS_Data[8];		//status gps fix
    ardu_gps_trans.buf[11] = GPS_Data[9];		//status flags
    ardu_gps_trans.buf[12] = GPS_Data[10];		//sol gps fix
    ardu_gps_trans.buf[13] = GPS_Data[11];		//sol flags
    I2CTransmit(ARDUIMU_I2C_DEV, ardu_gps_trans, ArduIMU_SLAVE_ADDR, 13);

    gps_data_status = GPS_DATA_MSG1;
  }

}

void ArduIMU_periodic( void ) {
  //Frequence defined in conf/modules/ins_arduimu.xml

  if (ardu_ins_trans.status == I2CTransDone) {
    I2CReceive(ARDUIMU_I2C_DEV, ardu_ins_trans, ArduIMU_SLAVE_ADDR, NB_DATA*2);
  }

  /*
     Buffer O:	Roll
     Buffer 1:	Pitch
     Buffer 2:	Yaw
     Buffer 3:	Gyro X
     Buffer 4:	Gyro Y
     Buffer 5:	Gyro Z
     Buffer 6:	Accel X
     Buffer 7:	Accel Y
     Buffer 8:	Accel Z
     */

}

#include "math/pprz_algebra_int.h"

void ArduIMU_event( void ) {
  // Handle INS I2C event
  if (ardu_ins_trans.status == I2CTransSuccess) {
    // received data from I2C transaction
    recievedData[0] = (ardu_ins_trans.buf[1]<<8) | ardu_ins_trans.buf[0];
    recievedData[1] = (ardu_ins_trans.buf[3]<<8) | ardu_ins_trans.buf[2];
    recievedData[2] = (ardu_ins_trans.buf[5]<<8) | ardu_ins_trans.buf[4];
    recievedData[3] = (ardu_ins_trans.buf[7]<<8) | ardu_ins_trans.buf[6];
    recievedData[4] = (ardu_ins_trans.buf[9]<<8) | ardu_ins_trans.buf[8];
    recievedData[5] = (ardu_ins_trans.buf[11]<<8) | ardu_ins_trans.buf[10];
    recievedData[6] = (ardu_ins_trans.buf[13]<<8) | ardu_ins_trans.buf[12];
    recievedData[7] = (ardu_ins_trans.buf[15]<<8) | ardu_ins_trans.buf[14];
    recievedData[8] = (ardu_ins_trans.buf[17]<<8) | ardu_ins_trans.buf[16];

    // Update ArduIMU data
    ArduIMU_data[0] = ANGLE_FLOAT_OF_BFP(recievedData[0]);
    ArduIMU_data[1] = ANGLE_FLOAT_OF_BFP(recievedData[1]);
    ArduIMU_data[2] = ANGLE_FLOAT_OF_BFP(recievedData[2]);
    ArduIMU_data[3] = RATE_FLOAT_OF_BFP(recievedData[3]);
    ArduIMU_data[4] = RATE_FLOAT_OF_BFP(recievedData[4]);
    ArduIMU_data[5] = RATE_FLOAT_OF_BFP(recievedData[5]);
    ArduIMU_data[6] = ACCEL_FLOAT_OF_BFP(recievedData[6]);
    ArduIMU_data[7] = ACCEL_FLOAT_OF_BFP(recievedData[7]);
    ArduIMU_data[8] = ACCEL_FLOAT_OF_BFP(recievedData[8]);

    // Update estimator
    estimator_phi = ArduIMU_data[0] - ins_roll_neutral;
    estimator_theta = ArduIMU_data[1] - ins_pitch_neutral;
    estimator_p = ArduIMU_data[3];
    //imu_daten_angefordert = FALSE;
    ardu_ins_trans.status = I2CTransDone;

    {
      float psi=0;
      float ax=ArduIMU_data[6];
      RunOnceEvery(15, DOWNLINK_SEND_AHRS_EULER(DefaultChannel, &estimator_phi, &estimator_theta, &psi));
      RunOnceEvery(15, DOWNLINK_SEND_IMU_ACCEL(DefaultChannel, &ax, &(ArduIMU_data[7]), &(ArduIMU_data[8])));
    }
  }
  else if (ardu_ins_trans.status == I2CTransFailed) {
    ardu_ins_trans.status = I2CTransDone;
  }
  // Handle GPS I2C event
  if (ardu_gps_trans.status == I2CTransSuccess || ardu_gps_trans.status == I2CTransFailed) {
    ardu_gps_trans.status = I2CTransDone;
  }
}

