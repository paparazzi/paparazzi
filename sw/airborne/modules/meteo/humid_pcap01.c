/*
 * Copyright (C) 2011 Norman Wildmann, Martin Mueller
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
 *
 */

/**
 * @file modules/meteo/humid_pcap01.c
 * @brief ACAM Picocap Single-chip Solution for Capacitance Measurement
 *
 * This reads the values for temperature and humidity from the ACAM capacitance and resistance
 * measurement unit through I2C.
 */

#include "led.h"
#include "mcu_periph/sys_time.h"
#include "mcu_periph/i2c.h"
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "modules/meteo/humid_pcap01.h"
#ifdef PCAP01_LOAD_FIRMWARE
#include "modules/meteo/humid_pcap01_firmware.h"
#endif


uint8_t  pcap01_meas_started;
struct i2c_transaction pcap01_trans;
PCAP01VALUE pcap01Value;

#ifndef PCAP01_I2C_DEV
#define PCAP01_I2C_DEV i2c0
#endif

void writePCAP01_SRAM(uint8_t data, uint16_t s_add)
{
  while (pcap01_trans.status == I2CTransPending);

  pcap01_trans.buf[0] = 0x90 + (unsigned char)(s_add >> 8);
  pcap01_trans.buf[1] = (unsigned char)(s_add);
  pcap01_trans.buf[2] = data;
  i2c_transmit(&PCAP01_I2C_DEV, &pcap01_trans, PCAP01_ADDR, 3);
}

uint8_t readPCAP01_SRAM(uint16_t s_add)
{
  while (pcap01_trans.status == I2CTransPending);

  pcap01_trans.buf[0] = 0x10 + (unsigned char)(s_add >> 8);
  pcap01_trans.buf[1] = (unsigned char)(s_add);
  i2c_transceive(&PCAP01_I2C_DEV, &pcap01_trans, PCAP01_ADDR, 2, 1);
  while (pcap01_trans.status == I2CTransPending);

  return pcap01_trans.buf[0];
}
/**
* \brief  PCAP01_Control
*
*         Function to send control commands to the PCAP01
*
* \param       control   Control command
*            possible commands:
*            PCAP01_PU_RESET : Hard reset of the device
*            PCAP01_IN_RESET : Software reset
*            PCAP01_START : Start measurement
*            PCAP01_START : Start measurement
*            PCAP01_TERM : Stop measurement
*/
void PCAP01_Control(uint8_t control)
{
  while (pcap01_trans.status == I2CTransPending);

  pcap01_trans.buf[0] = control;
  pcap01_trans.buf[1] = 0;
  pcap01_trans.buf[2] = 0;
  pcap01_trans.buf[3] = 0;
  i2c_transmit(&PCAP01_I2C_DEV, &pcap01_trans, PCAP01_ADDR, 4);
}

void pcap01writeRegister(uint8_t reg, uint32_t value)
{
  while (pcap01_trans.status == I2CTransPending);

  pcap01_trans.buf[0] = PCAP01_WRITE_REG + reg;
  pcap01_trans.buf[1] = (unsigned char)(value >> 16);
  pcap01_trans.buf[2] = (unsigned char)(value >> 8);
  pcap01_trans.buf[3] = (unsigned char)(value);
  i2c_transmit(&PCAP01_I2C_DEV, &pcap01_trans, PCAP01_ADDR, 4);
}

#ifdef PCAP01_LOAD_FIRMWARE
void writePCAP01_firmware(void)
{
  int i = 0;
  char testbyte = 44;
  uint16_t testaddress = 71;
  // Hard reset
  PCAP01_Control(PCAP01_PU_RESET);
  sys_time_usleep(50000);
  //write testbyte
  writePCAP01_SRAM(testbyte, testaddress);

  //check testbyte
  if (readPCAP01_SRAM(testaddress) != testbyte) {
    return;
  } else {
    LED_ON(3);
    //Hard reset
    PCAP01_Control(PCAP01_PU_RESET);
    //write firmware
    for (i = 0; i < sizeof(firmware); i++) {
      writePCAP01_SRAM(firmware[i], i);
    }
    //fill with ffs
    for (; i < 4029; i++) {
      writePCAP01_SRAM(0xff, i);
    }
    i++;
#ifdef PCAP01_200HZ
    //write end bytes of sram
    writePCAP01_SRAM(0x04, i++);
    writePCAP01_SRAM(0x01, i++);
    writePCAP01_SRAM(0x01, i++);
#endif
#ifdef PCAP01_STANDARD
    //write end bytes of sram
    writePCAP01_SRAM(0x02, i++);
    writePCAP01_SRAM(0x01, i++);
    writePCAP01_SRAM(0x03, i++);
#endif
  }
}
#endif // PCAP01_LOAD_FIRMWARE

void pcap01_init(void)
{
  pcap01_trans.status = I2CTransDone;
  pcap01Value.status = PCAP01_IDLE;
#ifdef PCAP01_LOAD_FIRMWARE
  writePCAP01_firmware();
#endif
  pcap01writeRegister(PCAP01_REG0, PCAP01_REG0_VALUE);
  pcap01writeRegister(PCAP01_REG1, PCAP01_REG1_VALUE);
  pcap01writeRegister(PCAP01_REG2, PCAP01_REG2_VALUE);
  pcap01writeRegister(PCAP01_REG3, PCAP01_REG3_VALUE);
  pcap01writeRegister(PCAP01_REG4, PCAP01_REG4_VALUE);
  pcap01writeRegister(PCAP01_REG5, PCAP01_REG5_VALUE);
  pcap01writeRegister(PCAP01_REG6, PCAP01_REG6_VALUE);
  pcap01writeRegister(PCAP01_REG7, PCAP01_REG7_VALUE);
  pcap01writeRegister(PCAP01_REG8, PCAP01_REG8_VALUE);
  pcap01writeRegister(PCAP01_REG9, PCAP01_REG9_VALUE);
  pcap01writeRegister(PCAP01_REG10, PCAP01_REG10_VALUE);
  pcap01writeRegister(PCAP01_REG11, PCAP01_REG11_VALUE);
  pcap01writeRegister(PCAP01_REG12, PCAP01_REG12_VALUE);
  pcap01writeRegister(PCAP01_REG13, PCAP01_REG13_VALUE);
  pcap01writeRegister(PCAP01_REG14, PCAP01_REG14_VALUE);
  pcap01writeRegister(PCAP01_REG15, PCAP01_REG15_VALUE);
  pcap01writeRegister(PCAP01_REG16, PCAP01_REG16_VALUE);
  pcap01writeRegister(PCAP01_REG17, PCAP01_REG17_VALUE);
  pcap01writeRegister(PCAP01_REG18, PCAP01_REG18_VALUE);
  pcap01writeRegister(PCAP01_REG19, PCAP01_REG19_VALUE);
  pcap01writeRegister(PCAP01_REG20, PCAP01_REG20_VALUE);
  PCAP01_Control(PCAP01_IN_RESET);
  sys_time_usleep(500000);
  PCAP01_Control(PCAP01_START);
}

void pcap01readRegister(uint8_t reg)
{
  uint16_t byte1 = 0x40 | reg;
  pcap01_trans.buf[0] = byte1;
  i2c_transceive(&PCAP01_I2C_DEV, &pcap01_trans, PCAP01_ADDR, 1, 3);
}

/**
* \brief  pcap01_readData
*
*         function where current measurement data from pcap01 is read into
*         global sensor variable
*/
void pcap01_periodic(void)
{
  pcap01Value.status = PCAP01_GET_HUMID;
#ifdef PCAP01_STANDARD
  pcap01readRegister(PCAP01_REG1);
#endif
#ifdef PCAP01_200HZ
  pcap01readRegister(PCAP01_REG2);
#endif
}

void pcap01_event(void)
{
  float humidity;
  float temperature;

  if (pcap01_trans.status == I2CTransSuccess) {
    switch (pcap01Value.status) {

      case PCAP01_GET_HUMID:
        pcap01Value.C_ratio = pcap01_trans.buf[0] << 16;
        pcap01Value.C_ratio |= (pcap01_trans.buf[1] << 8);
        pcap01Value.C_ratio |= pcap01_trans.buf[2];
        pcap01Value.status = PCAP01_GET_TEMP;
#ifdef PCAP01_STANDARD
        pcap01readRegister(PCAP01_REG13);
#endif
#ifdef PCAP01_200HZ
        pcap01readRegister(PCAP01_REG3);
#endif
        break;

      case PCAP01_GET_TEMP:
        pcap01Value.R_ratio = pcap01_trans.buf[0] << 16;
        pcap01Value.R_ratio |= (pcap01_trans.buf[1] << 8);
        pcap01Value.R_ratio |= pcap01_trans.buf[2];
        humidity = pcap01Value.C_ratio * (-0.0023959245437) + 516.4124438673063;
        temperature = pcap01Value.R_ratio * 61.927 - 259.74;
        DOWNLINK_SEND_PCAP01_STATUS(DefaultChannel, DefaultDevice,
                                    &pcap01Value.C_ratio,
                                    &pcap01Value.R_ratio,
                                    &humidity,
                                    &temperature);
        pcap01_trans.status = I2CTransDone;
        pcap01Value.status = PCAP01_IDLE;
        break;

      default:
        pcap01_trans.status = I2CTransDone;
        break;
    }
  }
}
