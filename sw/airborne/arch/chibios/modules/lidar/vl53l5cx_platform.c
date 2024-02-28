
/*
 * Copyright (C) 2024 Fabien-B <name.surname@gmail.com>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */


#include "lidar/vl53l5cx_platform.h"
#include "hal.h"

#define VL53L5_I2C_TIMEOUT chTimeMS2I(100)


uint8_t RdByte(
		VL53L5CX_Platform *dev,
		uint16_t index,
		uint8_t *p_data)
{
	return RdMulti(dev, index, p_data, 1);
}

uint8_t WrByte(
		VL53L5CX_Platform *dev,
		uint16_t index,
		uint8_t data)
{
	return WrMulti(dev, index, &data, 1);
}

uint8_t WrMulti(
		VL53L5CX_Platform *dev,
		uint16_t index,
		uint8_t *pdata,
		uint32_t count)
{
	I2CDriver * i2cd = (I2CDriver *)dev->i2cdev->reg_addr;

	i2cAcquireBus(i2cd);

	msg_t ret;

	for(size_t offset = 0; offset<count; offset += VL53L5CX_I2C_BUF_SIZE) {
		uint16_t reg_addr = index + offset;
		dev->buf[0] = (reg_addr & 0xFF00) >> 8; // MSB first
		dev->buf[1] = (reg_addr & 0x00FF);
		size_t size = Min(count-offset, VL53L5CX_I2C_BUF_SIZE);
		memcpy((uint8_t *) dev->buf + 2, (pdata + offset), size);
		cacheBufferFlush(dev->buf, size+2);
		ret = i2cMasterTransmitTimeout(i2cd, dev->address, dev->buf, size+2, NULL, 0, VL53L5_I2C_TIMEOUT);
	}


	i2cReleaseBus(i2cd);

	return ret;
}

uint8_t RdMulti(
		VL53L5CX_Platform *dev,
		uint16_t index,
		uint8_t *pdata,
		uint32_t count)
{
	I2CDriver * i2cd = (I2CDriver *)dev->i2cdev->reg_addr;
  
	i2cAcquireBus(i2cd);
	
	msg_t ret = 0;
	for(size_t offset = 0; offset<count; offset += VL53L5CX_I2C_BUF_SIZE) {
		uint16_t reg_addr = index + offset;
		dev->buf[0] = (reg_addr & 0xFF00) >> 8; // MSB first
		dev->buf[1] = (reg_addr & 0x00FF);
		size_t size = Min(count-offset, VL53L5CX_I2C_BUF_SIZE);

		cacheBufferFlush(dev->buf, 2);
		ret = i2cMasterTransmitTimeout(i2cd, dev->address, dev->buf, 2, dev->buf, size, VL53L5_I2C_TIMEOUT);
		cacheBufferInvalidate(dev->buf, count);
		memcpy(pdata+offset, dev->buf, count);
	}

	i2cReleaseBus(i2cd);

	return ret;
}

void SwapBuffer(
		uint8_t 		*buffer,
		uint16_t 	 	 size)
{
	uint32_t i, tmp;
	
	/* Example of possible implementation using <string.h> */
	for(i = 0; i < size; i = i + 4) 
	{
		tmp = (
		  buffer[i]<<24)
		|(buffer[i+1]<<16)
		|(buffer[i+2]<<8)
		|(buffer[i+3]);
		
		memcpy(&(buffer[i]), &tmp, 4);
	}
}	

uint8_t WaitMs(
		VL53L5CX_Platform *p_platform,
		uint32_t TimeMs)
{
	(void)p_platform;
	chThdSleepMilliseconds(TimeMs);
	return 0;
}
