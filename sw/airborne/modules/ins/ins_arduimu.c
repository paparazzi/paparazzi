/* 
C Datei für die Einbindung eines ArduIMU's
Autoren@ZHAW:   schmiemi
                chaneren
Reworked by S.Joyce, SmartPlanes AB
            Martin Mueller
*/


#include "modules/ins/ins_arduimu.h"
#include "mcu_periph/i2c.h"
#include "estimator.h"
#include "gps.h"

/* SCALE is deg/s/adc_unit

   LPR530 & LY530 Sensitivity (from datasheet)
   3.33mV/deg/s, 3.22mV/ADC step => 1.03
   Tested values : 0.96,0.96,0.94
*/

#ifndef GYRO_ROLL_DIRECTION
#define GYRO_ROLL_DIRECTION 1.
#endif
#ifndef GYRO_ROLL_SCALE
#define GYRO_ROLL_SCALE 0.96
#endif

#ifndef INS_FORMAT
#define INS_FORMAT float
#endif

#ifndef ARDUIMU_I2C_DEV
#define ARDUIMU_I2C_DEV i2c0
#endif

#define ArduIMU_SLAVE_ADDR 0x24

#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif

#include "mcu_periph/uart.h"
#include "messages.h"
#include "downlink.h"

struct i2c_transaction ardu_gps_trans;
struct i2c_transaction ardu_ins_trans;

int8_t ins_data_requested;

float ins_roll_neutral;
float ins_pitch_neutral;
float ins_yaw_neutral;

int renorm_sqrt_count;
int imu_overrun;
float imu_health;

void ins_ardu_init( void ) {

    ins_data_requested = FALSE;
    ins_roll_neutral = RadOfDeg(INS_ROLL_NEUTRAL_DEFAULT);
    ins_pitch_neutral = RadOfDeg(INS_PITCH_NEUTRAL_DEFAULT);
    ins_yaw_neutral = 0; //RadOfDeg(INS_YAW_NEUTRAL_DEFAULT);
}

void ins_ardu_event( void ) {

    static uint32_t gps_itow_prev = 0;

    /* check if gps data arrived from ublox, itow is last to be sent */
    if (gps_itow_prev != gps_itow) {
        gps_itow_prev = gps_itow;
        ins_ardu_send_gps();
    }

    /* check if data arrived from ArduIMU */
    if ((ardu_ins_trans.status == I2CTransSuccess) && 
        (ins_data_requested == TRUE)) {
      ins_data_process();
    }
}

void ins_ardu_send_gps( void ) {

    int32_t GPS_Data[5];

    //velned
    GPS_Data[0] = gps_gspeed;           //ground speed
    GPS_Data[1] = gps_course * 10000;   //heading, decideg
    //status/sol
    GPS_Data[2] = gps_mode;             //fix
    GPS_Data[3] = gps_sol_flags;        //flags
    //velned
    GPS_Data[4] = gps_speed_3d;         //speed 3D

    ardu_gps_trans.buf[0] = (uint8_t)  GPS_Data[0];     //ground speed
    ardu_gps_trans.buf[1] = (uint8_t) (GPS_Data[0] >>8);
    ardu_gps_trans.buf[2] = (uint8_t) (GPS_Data[0] >>16);
    ardu_gps_trans.buf[3] = (uint8_t) (GPS_Data[0] >>24);
    ardu_gps_trans.buf[4] = (uint8_t)  GPS_Data[1];     //ground course
    ardu_gps_trans.buf[5] = (uint8_t) (GPS_Data[1] >>8);
    ardu_gps_trans.buf[6] = (uint8_t) (GPS_Data[1] >>16);
    ardu_gps_trans.buf[7] = (uint8_t) (GPS_Data[1] >>24);
    ardu_gps_trans.buf[8] = (uint8_t)  GPS_Data[2];     //status fix
    ardu_gps_trans.buf[9] = (uint8_t)  GPS_Data[3];     //status flags
    ardu_gps_trans.buf[10] = (uint8_t)  GPS_Data[4];    //speed 3D
    ardu_gps_trans.buf[11] = (uint8_t) (GPS_Data[4] >>8);
    ardu_gps_trans.buf[12] = (uint8_t) (GPS_Data[4] >>16);
    ardu_gps_trans.buf[13] = (uint8_t) (GPS_Data[4] >>24);
    I2CTransmit(ARDUIMU_I2C_DEV, ardu_gps_trans, ArduIMU_SLAVE_ADDR, 14);
}

void ins_ardu_periodic( void ) {

    /* did we receive the previous packet? */
    if (ins_data_requested == TRUE) {	
        // ins_nb_err++;
    }
    I2CReceive(ARDUIMU_I2C_DEV, ardu_ins_trans, ArduIMU_SLAVE_ADDR, 8);
    ins_data_requested = TRUE;
}

void ins_data_process( void ) {

    int16_t receivedData[4];

    /* euler angles */
    INS_FORMAT ins_phi, ins_theta, ins_psi;

    /* gyro rates */
    INS_FORMAT ins_p, dummy = 0;

    receivedData[0] = (ardu_ins_trans.buf[1]<<8) | ardu_ins_trans.buf[0];
    receivedData[1] = (ardu_ins_trans.buf[3]<<8) | ardu_ins_trans.buf[2];
    receivedData[2] = (ardu_ins_trans.buf[5]<<8) | ardu_ins_trans.buf[4];
    receivedData[3] = (ardu_ins_trans.buf[7]<<8) | ardu_ins_trans.buf[6];

    ins_phi   = RadOfDeg(receivedData[0] / 100.) - ins_roll_neutral;
    ins_theta = RadOfDeg(receivedData[1] / 100.) - ins_pitch_neutral;
    ins_psi   = RadOfDeg(receivedData[2] / 100.) - ins_yaw_neutral;

    ins_p = GYRO_ROLL_DIRECTION * 
              RadOfADC(receivedData[3], GYRO_ROLL_SCALE);

    /* pitch/yaw rate and accelerator are not transferred */

#ifdef USE_INFRARED
#warning thermopile infrared can not be used with imu, DISABLING IMU 
#else
    EstimatorSetAtt(ins_phi, ins_psi, ins_theta);
    EstimatorSetRate(ins_p, dummy);
#endif

    RunOnceEvery(15,
        DOWNLINK_SEND_AHRS_EULER(DefaultChannel, &ins_phi, &ins_theta, &ins_psi));
    RunOnceEvery(6,
        DOWNLINK_SEND_IMU_GYRO(DefaultChannel, &ins_p, &dummy, &dummy));

    ins_data_requested = FALSE;
}
