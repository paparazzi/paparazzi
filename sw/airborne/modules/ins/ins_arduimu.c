/*
C Datei für die Einbindung eines ArduIMU's
Autoren@ZHAW:   schmiemi
        chaneren
*/


#include <stdbool.h>
#include "modules/ins/ins_arduimu.h"
#include "firmwares/fixedwing/main_fbw.h"
#include "mcu_periph/i2c.h"

// test
#include "state.h"

// für das Senden von GPS-Daten an den ArduIMU
#ifndef UBX
#error "currently only compatible with uBlox GPS modules"
#endif
#include "modules/gps/gps.h"
int32_t GPS_Data[14];

#ifndef ARDUIMU_I2C_DEV
#define ARDUIMU_I2C_DEV i2c0
#endif

// Adresse des I2C Slaves:  0001 0110 letztes Bit ist für Read/Write
// einzugebende Adresse im ArduIMU ist 0000 1011
//da ArduIMU das Read/Write Bit selber anfügt.
#define ArduIMU_SLAVE_ADDR 0x22


#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "subsystems/datalink/downlink.h"

struct i2c_transaction ardu_gps_trans;
struct i2c_transaction ardu_ins_trans;

static int16_t recievedData[NB_DATA];
float ArduIMU_data[NB_DATA];
int8_t messageNr;
int8_t imu_daten_angefordert;
int8_t gps_daten_abgespeichert;
int8_t gps_daten_versendet_msg1;
int8_t gps_daten_versendet_msg2;

float ins_roll_neutral;
float ins_pitch_neutral;
//float pitch_of_throttle_gain;
float throttle_slew;

void ArduIMU_init(void)
{
  ardu_gps_trans.buf[0] = 0;
  ardu_gps_trans.buf[1] = 0;
  messageNr = 0;
  imu_daten_angefordert = false;
  gps_daten_abgespeichert = false;
  gps_daten_versendet_msg1 = false;
  gps_daten_versendet_msg2 = false;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
//  pitch_of_throttle_gain = PITCH_OF_THROTTLE_GAIN;
  throttle_slew = V_CTL_THROTTLE_SLEW;
}


void ArduIMU_periodicGPS(void)
{

  if (gps_daten_versendet_msg1 == TRUE && gps_daten_versendet_msg2 == TRUE) {
    gps_daten_abgespeichert = false;
  }

  if (imu_daten_angefordert == TRUE) {
    IMU_Daten_verarbeiten();
  }

  if (gps_daten_abgespeichert == FALSE) {
    //posllh
    GPS_Data [0] = gps.tow;
    GPS_Data [1] = gps.lla_pos.lon;
    GPS_Data [2] = gps.lla_pos.lat;
    GPS_Data [3] = gps.lla_pos.alt;     //höhe über elipsoid
    GPS_Data [4] = gps.hmsl;    //höhe über sea level
    //velend
    GPS_Data [5] = gps.speed_3d;    //speed 3D
    GPS_Data [6] = gps.gspeed;    //ground speed
    GPS_Data [7] = DegOfRad(gps.course / 1e6);  //Kurs
    //status
    GPS_Data [8] = gps.fix;   //fix
    GPS_Data [9] = gps_ubx.status_flags;  //flags
    //sol
    GPS_Data [10] = gps.fix;    //fix
    //GPS_Data [11] = gps_ubx.sol_flags;    //flags
    GPS_Data [12] = -gps.ned_vel.z;
    GPS_Data [13] = gps.num_sv;

    gps_daten_abgespeichert = true;
  }

  if (messageNr == 0) {

    //test für 32bit in byte packete abzupacken:
    //GPS_Data [0] = -1550138773;

    ardu_gps_trans.buf[0] = 0;        //message Nr = 0 --> itow bis ground speed
    ardu_gps_trans.buf[1] = (uint8_t) GPS_Data[0];    //itow
    ardu_gps_trans.buf[2] = (uint8_t)(GPS_Data[0] >> 8);
    ardu_gps_trans.buf[3] = (uint8_t)(GPS_Data[0] >> 16);
    ardu_gps_trans.buf[4] = (uint8_t)(GPS_Data[0] >> 24);
    ardu_gps_trans.buf[5] = (uint8_t) GPS_Data[1];    //lon
    ardu_gps_trans.buf[6] = (uint8_t)(GPS_Data[1] >> 8);
    ardu_gps_trans.buf[7] = (uint8_t)(GPS_Data[1] >> 16);
    ardu_gps_trans.buf[8] = (uint8_t)(GPS_Data[1] >> 24);
    ardu_gps_trans.buf[9] = (uint8_t) GPS_Data[2];    //lat
    ardu_gps_trans.buf[10] = (uint8_t)(GPS_Data[2] >> 8);
    ardu_gps_trans.buf[11] = (uint8_t)(GPS_Data[2] >> 16);
    ardu_gps_trans.buf[12] = (uint8_t)(GPS_Data[2] >> 24);
    ardu_gps_trans.buf[13] = (uint8_t) GPS_Data[3];   //height
    ardu_gps_trans.buf[14] = (uint8_t)(GPS_Data[3] >> 8);
    ardu_gps_trans.buf[15] = (uint8_t)(GPS_Data[3] >> 16);
    ardu_gps_trans.buf[16] = (uint8_t)(GPS_Data[3] >> 24);
    ardu_gps_trans.buf[17] = (uint8_t) GPS_Data[4];   //hmsl
    ardu_gps_trans.buf[18] = (uint8_t)(GPS_Data[4] >> 8);
    ardu_gps_trans.buf[19] = (uint8_t)(GPS_Data[4] >> 16);
    ardu_gps_trans.buf[20] = (uint8_t)(GPS_Data[4] >> 24);
    ardu_gps_trans.buf[21] = (uint8_t) GPS_Data[5];   //speed
    ardu_gps_trans.buf[22] = (uint8_t)(GPS_Data[5] >> 8);
    ardu_gps_trans.buf[23] = (uint8_t)(GPS_Data[5] >> 16);
    ardu_gps_trans.buf[24] = (uint8_t)(GPS_Data[5] >> 24);
    ardu_gps_trans.buf[25] = (uint8_t) GPS_Data[6];   //gspeed
    ardu_gps_trans.buf[26] = (uint8_t)(GPS_Data[6] >> 8);
    ardu_gps_trans.buf[27] = (uint8_t)(GPS_Data[6] >> 16);
    ardu_gps_trans.buf[28] = (uint8_t)(GPS_Data[6] >> 24);
    i2c_transmit(&ARDUIMU_I2C_DEV, &ardu_gps_trans, ArduIMU_SLAVE_ADDR, 28);

    gps_daten_versendet_msg1 = true;
    messageNr = 1;
  } else {

    ardu_gps_trans.buf[0] = 1;      //message Nr = 1 --> ground course, ecefVZ, numSV, Fix, flags, fix, flags
    ardu_gps_trans.buf[1] = GPS_Data[7];    //ground course
    ardu_gps_trans.buf[2] = (GPS_Data[7] >> 8);
    ardu_gps_trans.buf[3] = (GPS_Data[7] >> 16);
    ardu_gps_trans.buf[4] = (GPS_Data[7] >> 24);
    ardu_gps_trans.buf[5] = GPS_Data[12];   //ecefVZ
    ardu_gps_trans.buf[6] = (GPS_Data[12] >> 8);
    ardu_gps_trans.buf[7] = (GPS_Data[12] >> 16);
    ardu_gps_trans.buf[8] = (GPS_Data[12] >> 24);
    ardu_gps_trans.buf[9] = GPS_Data[13];   //numSV
    ardu_gps_trans.buf[10] = GPS_Data[8];   //status gps fix
    ardu_gps_trans.buf[11] = GPS_Data[9];   //status flags
    ardu_gps_trans.buf[12] = GPS_Data[10];    //sol gps fix
    ardu_gps_trans.buf[13] = GPS_Data[11];    //sol flags
    i2c_transmit(&ARDUIMU_I2C_DEV, &ardu_gps_trans, ArduIMU_SLAVE_ADDR, 13);

    gps_daten_versendet_msg2 = true;
    messageNr = 0;
  }

  //DOWNLINK_SEND_DEBUG_ZHAW(DefaultChannel, DefaultDevice, &gps_mode , &gps_numSV ,&gps_alt , &gps_hmsl , &gps.tow, &gps_speed_3d);
}

void ArduIMU_periodic(void)
{
//Frequenz des Aufrufs wird in conf/modules/ArduIMU.xml festgelegt.

  //I2C-Nachricht anfordern an Slave Adresse, erwartete Anzahl Byte, Status
  if (imu_daten_angefordert == TRUE) {
    IMU_Daten_verarbeiten();
  }
  i2c_receive(&ARDUIMU_I2C_DEV, &ardu_ins_trans, ArduIMU_SLAVE_ADDR, 12);

  imu_daten_angefordert = true;
  /*
  Buffer O: Roll
  Buffer 1: Pitch
  Buffer 2: Yaw
  Buffer 3: Beschleunigung X-Achse
  Buffer 4: Beschleunigung Y-Achse
  Buffer 5: Beschleunigung Z-Achse
  */


  //Nachricht zum GCS senden
  // DOWNLINK_SEND_ArduIMU(DefaultChannel, DefaultDevice, &ArduIMU_data[0], &ArduIMU_data[1], &ArduIMU_data[2], &ArduIMU_data[3], &ArduIMU_data[4], &ArduIMU_data[5]);

  //    DOWNLINK_SEND_DEBUG_ZHAW(DefaultChannel, DefaultDevice, &airspeed_mode , &altitude_mode ,&amsys_baro, &amsys_baro, &amsys_airspeed_scaliert, &amsys_baro_scaliert);
}

void IMU_Daten_verarbeiten(void)
{
  //Empfangene Byts zusammenfügen und bereitstellen
  recievedData[0] = (ardu_ins_trans.buf[1] << 8) | ardu_ins_trans.buf[0];
  recievedData[1] = (ardu_ins_trans.buf[3] << 8) | ardu_ins_trans.buf[2];
  recievedData[2] = (ardu_ins_trans.buf[5] << 8) | ardu_ins_trans.buf[4];
  recievedData[3] = (ardu_ins_trans.buf[7] << 8) | ardu_ins_trans.buf[6];
  recievedData[4] = (ardu_ins_trans.buf[9] << 8) | ardu_ins_trans.buf[8];
  recievedData[5] = (ardu_ins_trans.buf[11] << 8) | ardu_ins_trans.buf[10];

  //Floats zurück transformieren. Transformation ist auf ArduIMU programmiert.
  ArduIMU_data[0] = (float)(recievedData[0] / (float)100);
  ArduIMU_data[1] = (float)(recievedData[1] / (float)100);
  ArduIMU_data[2] = (float)(recievedData[2] / (float)100);
  ArduIMU_data[3] = (float)(recievedData[3] / (float)100);
  ArduIMU_data[4] = (float)(recievedData[4] / (float)100);
  ArduIMU_data[5] = (float)(recievedData[5] / (float)100);

  // test
  struct FloatEulers att;
  att.phi = (float)ArduIMU_data[0] * 0.01745329252 - ins_roll_neutral;
  att.theta = (float)ArduIMU_data[1] * 0.01745329252 - ins_pitch_neutral;
  att.psi = 0.;
  imu_daten_angefordert = false;
  stateSetNedToBodyEulers_f(&att);
  uint8_t arduimu_id = 102;

  RunOnceEvery(15, DOWNLINK_SEND_AHRS_EULER(DefaultChannel, DefaultDevice, &att->phi, &att->theta, &att->psi, &arduimu_id));
}
