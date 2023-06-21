/*
 * SMS_STS.h
 * FeeTech SMS/STS series serial servo application layer program
 * Date: 2022.1.6
 * Author: 
 */

#ifndef _SMS_STS_H
#define _SMS_STS_H

#include <stdint.h>
#include "mcu_periph/uart.h"
#include "circular_buffer.h"

#ifndef STS3032_NB_SERVO
  #define STS3032_NB_SERVO 1
#endif

enum sts3032_rx_state {
  STS3032_RX_IDLE,
  STS3032_RX_HEAD_OK,
  STS3032_RX_GOT_LENGTH,
};

struct sts3032 {
  struct uart_periph *periph;

  
  uint8_t ids[STS3032_NB_SERVO];
  uint8_t states[STS3032_NB_SERVO];
  uint16_t pos[STS3032_NB_SERVO];

  uint32_t time_last_msg; // last send msg time


  enum sts3032_rx_state rx_state;
  uint8_t nb_bytes_expected;
  uint8_t buf_header[2];
  uint8_t rx_id;
  uint8_t rx_checksum;
  bool echo; // wait for sended message
  uint8_t read_addr;
  uint16_t nb_failed_checksum;

  struct cir_buf msg_buf;
};



#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_READ 0x82
#define INST_SYNC_WRITE 0x83


#define	SMS_STS_1M 		0
#define	SMS_STS_0_5M	1
#define	SMS_STS_250K	2
#define	SMS_STS_128K	3
#define	SMS_STS_115200	4
#define	SMS_STS_76800	5
#define	SMS_STS_57600	6
#define	SMS_STS_38400	7

//memory table definition
//-------EPROM(read only)--------
#define SMS_STS_MODEL_L	3
#define SMS_STS_MODEL_H	4

//-------EPROM(read and write)--------
#define SMS_STS_ID		5
#define SMS_STS_BAUD_RATE 6
#define SMS_STS_MIN_ANGLE_LIMIT_L 9
#define SMS_STS_MIN_ANGLE_LIMIT_H 10
#define SMS_STS_MAX_ANGLE_LIMIT_L 11
#define SMS_STS_MAX_ANGLE_LIMIT_H 12
#define SMS_STS_CW_DEAD	26
#define SMS_STS_CCW_DEAD 27
#define SMS_STS_OFS_L	31
#define SMS_STS_OFS_H	32
#define SMS_STS_MODE	33

//-------SRAM(read and write)--------
#define SMS_STS_TORQUE_ENABLE 40
#define SMS_STS_ACC		41
#define SMS_STS_GOAL_POSITION_L 42
#define SMS_STS_GOAL_POSITION_H 43
#define SMS_STS_GOAL_TIME_L 44
#define SMS_STS_GOAL_TIME_H 45
#define SMS_STS_GOAL_SPEED_L 46
#define SMS_STS_GOAL_SPEED_H 47
#define SMS_STS_LOCK	55

//-------SRAM(read only)--------
#define SMS_STS_PRESENT_POSITION_L 56
#define SMS_STS_PRESENT_POSITION_H 57
#define SMS_STS_PRESENT_SPEED_L 58
#define SMS_STS_PRESENT_SPEED_H 59
#define SMS_STS_PRESENT_LOAD_L 60
#define SMS_STS_PRESENT_LOAD_H 61
#define SMS_STS_PRESENT_VOLTAGE 62
#define SMS_STS_PRESENT_TEMPERATURE 63
#define SMS_STS_MOVING 66
#define SMS_STS_PRESENT_CURRENT_L 69
#define SMS_STS_PRESENT_CURRENT_H 70

void sts3032_write_pos(struct sts3032 *sts, uint8_t id, int16_t position);
void sts3032_event(struct sts3032 *sts);
void sts3032_read_mem(struct sts3032 *sts, uint8_t id, uint8_t addr, uint8_t len);
void sts3032_init(struct sts3032 *sts, struct uart_periph *periph, uint8_t* cbuf, size_t cbuf_len);
void sts3032_read_pos(struct sts3032 *sts, uint8_t id);
void sts3032_enable_torque(struct sts3032 *sts, uint8_t id, uint8_t enable);

extern int WritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC); //General write position command
extern int RegWritePosEx(uint8_t ID, int16_t Position, uint16_t Speed, uint8_t ACC); //Asynchronous write position command
extern void RegWriteAction(void); // Execute asynchronous written position
extern void SyncWritePosEx(uint8_t ID[], uint8_t IDN, int16_t Position[], uint16_t Speed[], uint8_t ACC[]); //Synchronous write position command
extern int WheelMode(uint8_t ID); //Constant speed mode
extern int WriteSpe(uint8_t ID, int16_t Speed, uint8_t ACC); //Constant speed mode control command
extern int EnableTorque(uint8_t ID, uint8_t Enable); //Torque control command
extern int unLockEprom(uint8_t ID); //EEPROM unlock
extern int LockEprom(uint8_t ID); //EEPROM lock
extern int CalibrationOfs(uint8_t ID); //Median calibration
extern int FeedBack(int ID); //Feedback servo information
extern int ReadPos(int ID); //Read position
extern int ReadSpeed(int ID); //Read speed
extern int ReadLoad(int ID); //Read output torque
extern int ReadVoltage(int ID); //Read voltage
extern int ReadTemper(int ID); //Read temperature
extern int ReadMove(int ID); //Read movement status
extern int ReadCurrent(int ID); //Read current
extern int getErr(void); //Return communication status, 0 no error, 1 communication error


#endif
