/*
 * Copyright (C) 2013-2020 Chris Efstathiou hendrixgr@gmail.com
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file modules/pca9685/pca9685.c
 * PCA9685 LED DRIVER USED AS A 16 ADDITIONAL PWM SERVO DRIVER.
 *
 */


//###################################################################################################
//                          I N C L U D E D   H E A D E R   F I L E S
//###################################################################################################
#include "modules/pca9685/pca9685_i2c.h"
#include "math/pprz_algebra_int.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/sys_time.h"

#include "math/pprz_isa.h"
#include "mcu_periph/sys_time.h"
#include "modules/core/abi.h"
#include "mcu_periph/uart.h"

#if DOWNLINK
#include "subsystems/datalink/telemetry.h"
#endif





//###################################################################################################
//          P R E P R O C E S S O R   D I R E C T I V E S   A N D   D E F I N I T I O N S
//###################################################################################################
// BIG ENDIAN DEFINITIONS
//#define BUF2INT(_buf,_idx) ((int16_t)((_buf[_idx]<<8) | _buf[_idx+1]))
//#define INT2BUF(_int,_buf,_idx) { _buf[_idx] = (_int>>8); _buf[_idx+1] = _int; }

// LITTLE ENDIAN DEFINITIONS USED FOR AvR CPU DUE TO MORE EFFICIENT DATA TRANSFERS
#define BUF2INT(_buf,_idx) ((int16_t)((_buf[_idx+1]<<8) | _buf[_idx]))
#define INT2BUF(_int,_buf,_idx) { _buf[_idx] = _int; _buf[_idx+1] = (_int>>8); }

#ifndef PCA9685_I2C_DEV
#define PCA9685_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(PCA9685_I2C_DEV)

#ifndef PCA9865_SRV_RESOLUTION
#define PCA9865_SRV_RESOLUTION    1. // in microseconds
#endif

#ifndef PCA9865_SRV_DEFAULT_VAL_US
#define PCA9865_SRV_DEFAULT_VAL_US  1000. // in microseconds
#endif

#ifndef PCA9865_SRV_NUMBER
#define PCA9865_SRV_NUMBER    16 // in microseconds
#endif

// I2C Addreses
#ifndef PCA9685_I2C_SLAVE_ADDR
#define PCA9685_I2C_SLAVE_ADDR    0xE0
#endif
PRINT_CONFIG_VAR(PCA9685_I2C_SLAVE_ADDR)

#ifndef PCA9685_I2C_ALLCALL_ADDR
#define PCA9685_I2C_ALLCALL_ADDR  0xE0
#endif
#ifndef PCA9685_I2C_RESET_ADDR
#define PCA9685_I2C_RESET_ADDR    0x06
#endif
#ifndef PCA9685_I2C_GEN_CALL_ADDR
#define PCA9685_I2C_GEN_CALL_ADDR 0x00
#endif

// Register addresses
#define PCA9685_MODE1_REG_ADDR    0x00
#define PCA9685_MODE2_REG_ADDR    0x01
#define PCA9685_ALLCALL_ADDR    0x05

#define PCA9685_LED0_ON_L_REG_ADDR  0X06
#define PCA9685_LED0_ON_H_REG_ADDR  0X07
#define PCA9685_LED0_OFF_L_REG_ADDR 0X08
#define PCA9685_LED0_OFF_H_REG_ADDR 0X09

#define PCA9685_LED1_ON_L_REG_ADDR  0X0A
#define PCA9685_LED1_ON_H_REG_ADDR  0X0B
#define PCA9685_LED1_OFF_L_REG_ADDR 0X0C
#define PCA9685_LED1_OFF_H_REG_ADDR 0X0D

#define PCA9685_LED2_ON_L_REG_ADDR  0X0E
#define PCA9685_LED2_ON_H_REG_ADDR  0X0F
#define PCA9685_LED2_OFF_L_REG_ADDR 0X10
#define PCA9685_LED2_OFF_H_REG_ADDR 0X11

#define PCA9685_LED3_ON_L_REG_ADDR  0X12
#define PCA9685_LED3_ON_H_REG_ADDR  0X13
#define PCA9685_LED3_OFF_L_REG_ADDR 0X14
#define PCA9685_LED3_OFF_H_REG_ADDR 0X15

#define PCA9685_LED4_ON_L_REG_ADDR  0X16
#define PCA9685_LED4_ON_H_REG_ADDR  0X17
#define PCA9685_LED4_OFF_L_REG_ADDR 0X18
#define PCA9685_LED4_OFF_H_REG_ADDR 0X19

#define PCA9685_LED5_ON_L_REG_ADDR  0X1A
#define PCA9685_LED5_ON_H_REG_ADDR  0X1B
#define PCA9685_LED5_OFF_L_REG_ADDR 0X1C
#define PCA9685_LED5_OFF_H_REG_ADDR 0X1D

#define PCA9685_LED6_ON_L_REG_ADDR  0X1E
#define PCA9685_LED6_ON_H_REG_ADDR  0X1F
#define PCA9685_LED6_OFF_L_REG_ADDR 0X20
#define PCA9685_LED6_OFF_H_REG_ADDR 0X21

#define PCA9685_LED7_ON_L_REG_ADDR  0X22
#define PCA9685_LED7_ON_H_REG_ADDR  0X23
#define PCA9685_LED7_OFF_L_REG_ADDR 0X24
#define PCA9685_LED7_OFF_H_REG_ADDR 0X25


#define PCA9685_LED8_ON_L_REG_ADDR  0X26
#define PCA9685_LED8_ON_H_REG_ADDR  0X27
#define PCA9685_LED8_OFF_L_REG_ADDR 0X28
#define PCA9685_LED8_OFF_H_REG_ADDR 0X29

#define PCA9685_LED9_ON_L_REG_ADDR  0X2A
#define PCA9685_LED9_ON_H_REG_ADDR  0X2B
#define PCA9685_LED9_OFF_L_REG_ADDR 0X2C
#define PCA9685_LED9_OFF_H_REG_ADDR 0X2D

#define PCA9685_LED10_ON_L_REG_ADDR 0X2E
#define PCA9685_LED10_ON_H_REG_ADDR 0X2F
#define PCA9685_LED10_OFF_L_REG_ADDR  0X30
#define PCA9685_LED10_OFF_H_REG_ADDR  0X31

#define PCA9685_LED11_ON_L_REG_ADDR 0X32
#define PCA9685_LED11_ON_H_REG_ADDR 0X33
#define PCA9685_LED11_OFF_L_REG_ADDR  0X34
#define PCA9685_LED11_OFF_H_REG_ADDR  0X35

#define PCA9685_LED12_ON_L_REG_ADDR 0X36
#define PCA9685_LED12_ON_H_REG_ADDR 0X37
#define PCA9685_LED12_OFF_L_REG_ADDR  0X38
#define PCA9685_LED12_OFF_H_REG_ADDR  0X39

#define PCA9685_LED13_ON_L_REG_ADDR 0X3A
#define PCA9685_LED13_ON_H_REG_ADDR 0X3B
#define PCA9685_LED13_OFF_L_REG_ADDR  0X3C
#define PCA9685_LED13_OFF_H_REG_ADDR  0X3D

#define PCA9685_LED14_ON_L_REG_ADDR 0X3E
#define PCA9685_LED14_ON_H_REG_ADDR 0X3F
#define PCA9685_LED14_OFF_L_REG_ADDR  0X40
#define PCA9685_LED14_OFF_H_REG_ADDR  0X41

#define PCA9685_LED15_ON_L_REG_ADDR 0X42
#define PCA9685_LED15_ON_H_REG_ADDR 0X43
#define PCA9685_LED15_OFF_L_REG_ADDR  0X44
#define PCA9685_LED15_OFF_H_REG_ADDR  0X45

#define PCA9685_ALL_LED_ON_L_REG_ADDR 0XFA
#define PCA9685_ALL_LED_ON_H_REG_ADDR 0XFB
#define PCA9685_ALL_LED_OFF_L_REG_ADDR  0XFC
#define PCA9685_ALL_LED_OFF_H_REG_ADDR  0XFD

#define PCA9685_PRESCALER_REG_ADDR  0XFE

//Bit positions
//MODE 0 REGISTER
#define PCA9865_RESTART_BIT   7 // 1= RESTART ENABLE, DEFAULT = 0
#define PCA9865_AUTO_INCREMENT_BIT  5 // 1 = AUTO INCREMENT, DEFAULT = 0
#define PCA9865_SLEEP_BIT   4 // 1 = LOW POWER MODE (OSC OFF), 0=NORMAL MODE, DEFAULT=0
#define PCA9865_ALLCALL_BIT   0 // 1 = ALL LED ENABLE, DEFAULT = 1
//MODE 1 REGISTER
#define PCA9865_OUTDRV_BIT    2 // 1 = TOTEM POLE, 0 = OPEN DRAIN, DEFAULT = 1




#if !defined(SITL)
//###################################################################################################
//                        P R I V A T E   F U N C T I O N   P R O T O T Y P E S
//###################################################################################################
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
static void pca9685_send_servo_values(struct transport_tx *trans, struct link_device *dev);
#endif




//###################################################################################################
//                               G L O B A L   V A R I A B L E S
//###################################################################################################

enum {
  PCA9685_I2C_STATUS_UNINIT,
  PCA9685_I2C_STATUS_INITIALIZED,
  PCA9865_I2C_STATUS_WRITE_LED_REG_BUSY,
  PCA9865_I2C_STATUS_WRITE_LED_REG_READY,
  PCA9865_I2C_STATUS_READ_LED_REG_BUSY,
  PCA9865_I2C_STATUS_CHANGE_LED_REG_FINISHED
};

struct   i2c_transaction pca9685_i2c_trans;
uint8_t  pca9685_i2c_status = 0;
uint8_t  srv_cnt = 0;
uint16_t pca9865_write_servo_vals[PCA9865_SRV_NUMBER];
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
PRINT_CONFIG_MSG("PCA9685 servo values can be found in the ""BARO_WORDS"" message")
uint16_t pca9865_read_servo_vals[PCA9865_SRV_NUMBER];
#endif
uint8_t  pca9865_reg_nb[16];


//###################################################################################################
//                         P R I V A T E   F U N C T I O N   D E F I N I T I O N S
//###################################################################################################

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
static void pca9685_send_servo_values(struct transport_tx *trans, struct link_device *dev)
{

// For debugging purposes we send anly the first 4 servo values using the BARO_WORDS message.
// Those values are read back from PCA9865 so they indicate if the module is working as intended.
  pprz_msg_send_CSC_SERVO_CMD(
    trans, dev, AC_ID,
    &pca9865_read_servo_vals[0], &pca9865_read_servo_vals[1],
    &pca9865_read_servo_vals[2], &pca9865_read_servo_vals[3]
  );


  return;
}
#endif


//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
//                        P U B L I C   F U N C T I O N   D E F I N I T I O N S
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
bool pca9865_set_servo(uint8_t srv_nb, uint16_t srv_val)
{
  uint16_t servo_value = 0;

  if (srv_nb < 16) {
    servo_value = (uint16_t)((float)srv_val / (float)PCA9865_SRV_RESOLUTION);
    pca9865_write_servo_vals[srv_nb] = servo_value;
    pca9865_reg_nb[srv_nb] = (srv_nb * 4) + 8;

  }

  return (FALSE);
}



//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
void pca9685_i2c_init(void)
{

  uint16_t reg_value = 0;
  uint32_t timer0 = 0;

// clear all arrays to zero.
  for (srv_cnt = 0; srv_cnt < PCA9865_SRV_NUMBER; srv_cnt++) {
    pca9865_write_servo_vals[srv_cnt] = 0;
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
    pca9865_read_servo_vals[srv_cnt] = 0;
#endif
    pca9865_reg_nb[srv_cnt] = 0;
  }
  srv_cnt = 0;
  pca9685_i2c_trans.status = I2CTransDone;
  pca9685_i2c_status = PCA9685_I2C_STATUS_UNINIT;

  pca9685_i2c_trans.buf[0] = PCA9685_I2C_RESET_ADDR;
  i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_GEN_CALL_ADDR, 1);
  timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
  while (pca9685_i2c_trans.status == I2CTransPending) {
    if (sys_time.nb_tick > timer0) { break; }
  }

// Internal Oscillator Off, the oscillator is off by default on power up.
  pca9685_i2c_trans.buf[0] = PCA9685_MODE1_REG_ADDR;
  pca9685_i2c_trans.buf[1] = (1 << PCA9865_SLEEP_BIT) | (1 << PCA9865_ALLCALL_BIT); // OSC off
  i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 2);
  timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
  while (pca9685_i2c_trans.status == I2CTransPending) {
    if (sys_time.nb_tick > timer0) { break; }
  }

  reg_value = (uint16_t)((float)PCA9865_SRV_RESOLUTION / 0.04); //0.04 = 1/25 Mhz
  if (reg_value > 0xFF) { reg_value = 0xFF; } // Sanity check.
  pca9685_i2c_trans.buf[0] = PCA9685_PRESCALER_REG_ADDR;
  pca9685_i2c_trans.buf[1] = (uint8_t)reg_value;
  i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 2);
  timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
  while (pca9685_i2c_trans.status == I2CTransPending) {
    if (sys_time.nb_tick > timer0) { break; }
  }

// Internal Oscillator On, Auto Increment on, AllCall enabled, 500 microseconds needed.
  pca9685_i2c_trans.buf[0] = PCA9685_MODE1_REG_ADDR;
  pca9685_i2c_trans.buf[1] = (1 << PCA9865_AUTO_INCREMENT_BIT) | (1 << PCA9865_ALLCALL_BIT);
  i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 2);
  timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
  while (pca9685_i2c_trans.status == I2CTransPending) {
    if (sys_time.nb_tick > timer0) { break; }
  }

  do {
    pca9685_i2c_trans.buf[0] = PCA9685_MODE1_REG_ADDR;
    i2c_transceive(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 1, 1);
    timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
    while (pca9685_i2c_trans.status == I2CTransPending) {
      if (sys_time.nb_tick > timer0) { break; }
    }

  } while ((pca9685_i2c_trans.buf[0] & (1 << PCA9865_RESTART_BIT)) != 0);

  pca9685_i2c_trans.buf[0] = PCA9685_ALL_LED_ON_L_REG_ADDR;
  pca9685_i2c_trans.buf[1] = 0;
  pca9685_i2c_trans.buf[2] = 0;
  reg_value = (uint16_t)((float)PCA9865_SRV_DEFAULT_VAL_US / (float)PCA9865_SRV_RESOLUTION);
  pca9685_i2c_trans.buf[3] = (uint8_t)reg_value;
  pca9685_i2c_trans.buf[4] = (uint8_t)((reg_value >> 8) & 0x000F);
  i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_ALLCALL_ADDR, 5);
  timer0 = sys_time.nb_tick + (SYS_TIME_FREQUENCY / 2);
  while (pca9685_i2c_trans.status == I2CTransPending) {
    if (sys_time.nb_tick > timer0) { break; }
  }


  pca9685_i2c_trans.status = I2CTransDone;
  pca9685_i2c_status = PCA9685_I2C_STATUS_INITIALIZED;

#if DOWNLINK
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_CSC_SERVO_CMD, pca9685_send_servo_values);
#endif
#endif

// Initialize all 16 servo with the default values
  pca9865_set_servo(0, 1500);
  pca9865_set_servo(1, 1500);
  pca9865_set_servo(2, 1000);
  pca9865_set_servo(3, 1500);
  for (srv_cnt = 4; srv_cnt < PCA9865_SRV_NUMBER; srv_cnt++) {
    pca9865_set_servo(srv_cnt, 1000);
  }
  srv_cnt = 0;

  return;
}



//333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
void pca9685_i2c_periodic(void)
{
// If a servo change was requested write the appropriate pca9685 registers.
// Since those servos are not intented for flight surface control they can be updated slower
// at about one servo each module cycle. This way i2c load is minimized.
//
// Start searching for a servo change request either after intialization or after all
// servo channels have been searched (looping).
// This loop will be executed always until a servo value change is requested.
// Then it will break with "srv_cnt" having the servo number to be changed.

// DELAY IN ORDER TO GIVE TIME TO THE INTERNAL PCA9865 OSCILLATOR TO STABILIZE.
//if (sys_time.nb_sec > 3){
  if (srv_cnt >= 16 || pca9685_i2c_status == PCA9685_I2C_STATUS_INITIALIZED) {
    for (srv_cnt = 0; srv_cnt < 16; srv_cnt++) {
      if (pca9865_reg_nb[srv_cnt] > 0) {
        pca9685_i2c_status = PCA9865_I2C_STATUS_CHANGE_LED_REG_FINISHED;
        break;
      }
    }
  }

  // Change the above found register in order to update the servo pwm value.
  // The search for another servo to change is completed in the "event" function.
  if (srv_cnt < 16) {
    if (pca9685_i2c_status == PCA9865_I2C_STATUS_CHANGE_LED_REG_FINISHED) {
      pca9685_i2c_trans.buf[0] = pca9865_reg_nb[srv_cnt];
      pca9685_i2c_trans.buf[1] = (uint8_t)pca9865_write_servo_vals[srv_cnt];
      pca9685_i2c_trans.buf[2] = (uint8_t)(pca9865_write_servo_vals[srv_cnt] >> 8) & 0x0F;
      i2c_transmit(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 3);
      pca9685_i2c_status = PCA9865_I2C_STATUS_WRITE_LED_REG_BUSY;
    }
  }
//}

  return;
}



//444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444
void pca9685_i2c_event(void)
{

//--------------------------------------------------------------------------------------------------
//                                   IF SUCCESFULL
//--------------------------------------------------------------------------------------------------
  if (pca9685_i2c_trans.status == I2CTransSuccess) {

    if (pca9685_i2c_status == PCA9865_I2C_STATUS_WRITE_LED_REG_BUSY) {
      // Read back the changed value just to be sure...
      pca9685_i2c_trans.buf[0] = pca9865_reg_nb[srv_cnt];
      i2c_transceive(&PCA9685_I2C_DEV, &pca9685_i2c_trans, PCA9685_I2C_SLAVE_ADDR, 1, 2);
      pca9685_i2c_status = PCA9865_I2C_STATUS_READ_LED_REG_BUSY;

    } else if (pca9685_i2c_status == PCA9865_I2C_STATUS_READ_LED_REG_BUSY) {
#if defined(PCA9685_SEND_SERVO_VALUES) && PCA9685_SEND_SERVO_VALUES == 1
      pca9865_read_servo_vals[srv_cnt] = BUF2INT(pca9685_i2c_trans.buf, 0);
#endif
      if (BUF2INT(pca9685_i2c_trans.buf, 0) != pca9865_write_servo_vals[srv_cnt]) {
        //pca9685_i2c_init();
        return;
      }
      pca9685_i2c_status = PCA9865_I2C_STATUS_CHANGE_LED_REG_FINISHED;
      // Delete this request.
      pca9865_reg_nb[srv_cnt] = 0;
      // Search for the next changed servo value.
      // If a value > 0 is detected x will contain the new servo number to be changed.
      srv_cnt++;
      for (; srv_cnt < 16; srv_cnt++) { if (pca9865_reg_nb[srv_cnt] > 0) { break; } }
    }
//-------------------------------------------------------------------------------------------------
//                              ELSE IF NOT SUCCESFULL
//-------------------------------------------------------------------------------------------------
  }//else{ pca9685_i2c_init(); }

  return;
}

#else
// just to stop the compiler complaining i manipulte srv_nb and srv_val a little.
bool pca9865_set_servo(uint8_t srv_nb, uint16_t srv_val) { srv_nb = srv_val; srv_nb /= 2; return (FALSE); }
void pca9685_i2c_init(void) { return; }
void pca9685_i2c_periodic(void) { return; }
void pca9685_i2c_event(void) { return; }

#endif // #if !defined(SITL)

/************************************************************************************************/
//                                       T H E   E N D
/************************************************************************************************/

