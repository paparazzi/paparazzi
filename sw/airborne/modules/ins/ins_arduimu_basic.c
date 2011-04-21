/*
C Datei für die Einbindung eines ArduIMU's
Autoren@ZHAW:   schmiemi
        chaneren
*/


#include <stdbool.h>
#include "modules/ins/ins_arduimu_basic.h"
//#include "firmwares/fixedwing/main_fbw.h"
#include "mcu_periph/i2c.h"

// test
#include "estimator.h"

// für das Senden von GPS-Daten an den ArduIMU
#include "gps.h"

#define NB_DATA 9

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

struct FloatEulers arduimu_eulers;
struct FloatRates arduimu_rates;
struct FloatVect3 arduimu_accel;

float ins_roll_neutral;
float ins_pitch_neutral;

void ArduIMU_init( void ) {
  FLOAT_EULERS_ZERO(arduimu_eulers);
  FLOAT_RATES_ZERO(arduimu_rates);
  FLOAT_VECT3_ZERO(arduimu_accel);

  ardu_ins_trans.status = I2CTransDone;
  ardu_gps_trans.status = I2CTransDone;

  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;
  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
}

#define FillBufWith32bit(_buf, _index, _value) {  \
  _buf[_index] = (uint8_t) (_value);              \
  _buf[_index+1] = (uint8_t) ((_value) >> 8);     \
  _buf[_index+2] = (uint8_t) ((_value) >> 16);    \
  _buf[_index+3] = (uint8_t) ((_value) >> 24);    \
}

void ArduIMU_periodicGPS( void ) {

  if (ardu_gps_trans.status != I2CTransDone) { return; }

  FillBufWith32bit(ardu_gps_trans.buf, 0, (int32_t)gps_speed_3d); // speed 3D
  FillBufWith32bit(ardu_gps_trans.buf, 4, (int32_t)gps_gspeed);   // ground speed
  FillBufWith32bit(ardu_gps_trans.buf, 8, (int32_t)gps_course);   // course
  ardu_gps_trans.buf[12] = gps_mode;                              // status gps fix
  ardu_gps_trans.buf[13] = gps_status_flags;                      // status flags
  I2CTransmit(ARDUIMU_I2C_DEV, ardu_gps_trans, ArduIMU_SLAVE_ADDR, 14);

}

void ArduIMU_periodic( void ) {
  //Frequence defined in conf/modules/ins_arduimu.xml

  if (ardu_ins_trans.status == I2CTransDone) {
    I2CReceive(ARDUIMU_I2C_DEV, ardu_ins_trans, ArduIMU_SLAVE_ADDR, NB_DATA*2);
  }

}

#include "math/pprz_algebra_int.h"
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
    arduimu_eulers.phi = ANGLE_FLOAT_OF_BFP(recievedData[0]);
    arduimu_eulers.theta = ANGLE_FLOAT_OF_BFP(recievedData[1]);
    arduimu_eulers.psi = ANGLE_FLOAT_OF_BFP(recievedData[2]);
    arduimu_rates.p = RATE_FLOAT_OF_BFP(recievedData[3]);
    arduimu_rates.q = RATE_FLOAT_OF_BFP(recievedData[4]);
    arduimu_rates.r = RATE_FLOAT_OF_BFP(recievedData[5]);
    arduimu_accel.x = ACCEL_FLOAT_OF_BFP(recievedData[6]);
    arduimu_accel.y = ACCEL_FLOAT_OF_BFP(recievedData[7]);
    arduimu_accel.z = ACCEL_FLOAT_OF_BFP(recievedData[8]);

    // Update estimator
    estimator_phi = arduimu_eulers.phi - ins_roll_neutral;
    estimator_theta = arduimu_eulers.theta - ins_pitch_neutral;
    estimator_p = arduimu_rates.p;
    ardu_ins_trans.status = I2CTransDone;

    //RunOnceEvery(15, DOWNLINK_SEND_AHRS_EULER(DefaultChannel, &arduimu_eulers.phi, &arduimu_eulers.theta, &arduimu_eulers.psi));
    RunOnceEvery(15, DOWNLINK_SEND_IMU_GYRO(DefaultChannel, &arduimu_rates.p, &arduimu_rates.q, &arduimu_rates.r));
    RunOnceEvery(15, DOWNLINK_SEND_IMU_ACCEL(DefaultChannel, &arduimu_accel.x, &arduimu_accel.y, &arduimu_accel.z));
  }
  else if (ardu_ins_trans.status == I2CTransFailed) {
    ardu_ins_trans.status = I2CTransDone;
  }
  // Handle GPS I2C event
  if (ardu_gps_trans.status == I2CTransSuccess || ardu_gps_trans.status == I2CTransFailed) {
    ardu_gps_trans.status = I2CTransDone;
  }
}

