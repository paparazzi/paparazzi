#ifndef CSC_MSG_DEF_H
#define CSC_MSG_DEF_H

#include "paparazzi.h"

#define CSC_GPS_AXIS_IDX_X		0
#define CSC_GPS_AXIS_IDX_Y		1
#define CSC_GPS_AXIS_IDX_Z		2

typedef enum { 
	CSC_SERVO_CMD_ID 				=  0, 
	CSC_MOTOR_CMD_ID 				=  1, 
	CSC_PROP_CMD_ID 				=  2, 
	CSC_MOTOR_STATUS_ID			=  3, 
	CSC_BOARD_STATUS_ID			=  4, 
	CSC_BOARD_ADCVOLTS_ID		=  5, 
	CSC_RC_ID								=  6, 
	CSC_GPS_FIX_ID					=  7, 
	CSC_GPS_POS_ID					=  8, 
	CSC_GPS_ACC_ID					=  9, 
	CSC_PROP2_CMD_ID				= 10, 
	CSC_VANE_MSG_ID					= 11, 
	CSC_PRESSURE_MSG_ID			= 12, 
	CSC_BARO_MSG_ID 				= 13, 
	CSC_BAT_MSG_ID 					= 14, 
	CSC_AIRSPEED_MSG_ID 		= 15, 
	
	CSC_ID_COUNT
} csc_can_msg_id;

/* Received from the autopilot */
struct CscServoCmd {
  uint16_t servos[4];
} __attribute__((packed));

/* For simple blades which have a positive speed,
   and variable pitch blades. */
struct CscPropCmd {
  uint8_t speeds[4];
} __attribute__((packed));

/* Send and Received between autopilot and csc */
struct CscMotorMsg {
  uint8_t  cmd_id;
  uint16_t arg1;
  uint16_t arg2;
} __attribute__((packed));

struct CscStatusMsg {
  uint32_t loop_count;
  uint32_t msg_count;
} __attribute__((packed));

struct CscADCMsg {
  float ADCVolts1;
  float ADCVolts2;
} __attribute__((packed));

struct CscRCMsg {
  uint16_t right_stick_vertical;
  uint16_t right_stick_horizontal;
  uint16_t left_stick_horizontal_and_aux2;
  uint16_t left_stick_vertical_and_flap_mix;
} __attribute__((packed));

struct CscGPSFixMsg {
  uint8_t gps_fix;
  uint8_t num_sv;
  int16_t vx;
  int16_t vy;
  int16_t vz;
} __attribute__((packed));

struct CscGPSPosMsg {
  int32_t val;
  uint8_t axis;
} __attribute__((packed));

struct CscGPSAccMsg {
  uint32_t pacc;
  uint32_t sacc;
} __attribute__((packed));

struct CscVaneMsg {
  float vane_angle1;
  float vane_angle2;
} __attribute__((packed));

struct CscPressureMsg {
  float pressure1;
  float pressure2;
} __attribute__((packed));

/*
struct CscAirspeedMsg {
  float airspeed1;
  float airspeed2;
} __attribute__((packed));
*/

struct CscAirspeedMsg {
  float airspeed;
  uint8_t sensor_addr;
} __attribute__((packed));

struct CscBatMsg {
  uint16_t volts;
  uint16_t amps;
  uint8_t msgctr;
} __attribute__((packed));

struct CscBaroMsg {
  uint32_t baro_pressure;
  uint16_t baro_temperature;
  uint8_t baro_status;
  uint8_t baro_sensor_addr;
} __attribute__((packed));

#define CSC_RC_SCALE 20
#define CSC_RC_OFFSET 2*(MAX_PPRZ/CSC_RC_SCALE) /* Sorry this is a bit arbitrary. - mmt */

#endif
