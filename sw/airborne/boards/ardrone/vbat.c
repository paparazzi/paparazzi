/*
    vbat.c - AR.Drone battery voltage driver

    Copyright (C) 2011 Hugo Perquin - http://blog.perquin.com

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/time.h>
#include <unistd.h>
#include "i2c-dev.h"
#include "vbat.h"

#define VBAT_ADDRESS 0x49

int fd;

float vbat_get(unsigned char channel) 
{
	if(channel>9) return -1;

	unsigned lower = i2c_smbus_read_byte_data(fd, 0x34 + (channel<<1));
	unsigned upper = i2c_smbus_read_byte_data(fd, 0x33 + (channel<<1));
	
	unsigned value = upper<<2 | lower;
	//VREF Reference Voltage Internally connected to VDDC pin. 1.8V +/- 0.05V
	//Measured Input Scaling Factor External inputs ANA{0,1,2,3}  0.25  V/V  	ch 0-3
	//Measured Input Scaling Factor VDD{0,1,2,3,4} inputs 0.4  V/V 				ch 4-8
	//Measured Input Scaling Factor VINSYS input 0.25 V/V 						ch 9
	float factor;
	if(channel<4) factor=0.031;
	else if(channel<9) factor=1.8/0.4/1023;
	else factor=1.8/0.25/1023;
	
	float v = value * factor;
	//printf("Channel=%d Vbat=%.2fVolt RawValue=%d RawHiByte=0x%x RawLiByte=0x%x\n",(int)channel,v,value,upper,lower);
	
	return v;
}

int vbat_init(vbat_struct *vbat)
{
	fd = open( "/dev/i2c-3", O_RDWR );

	if( ioctl( fd, I2C_SLAVE, VBAT_ADDRESS ) < 0 )
	{
	fprintf( stderr, "Failed to set slave address: %m\n" );
	return 1;
	}

	//ADC_CTRL
	if( i2c_smbus_write_byte_data( fd, 0x30, 0xc7))   {	
		fprintf( stderr, "Failed to write to I2C device 2\n" );
		return 2;
	}
	//ADC_MUX_1
	if( i2c_smbus_write_byte_data( fd, 0x31, 0x5f)){	
		fprintf( stderr, "Failed to write to I2C device 3\n" );
		return 3;
	} 
	//ADC_MUX_2
	if( i2c_smbus_write_byte_data( fd, 0x32, 0x0f))  {	
		fprintf( stderr, "Failed to write to I2C device 4\n" );
		return 4;
	}

	//Setpoint
	unsigned char v;
	v = i2c_smbus_read_byte_data(fd, 0x06);
	vbat->vdd0_setpoint = (v&0x80 ? (v & 0x3f) * 0.05 + 0.80 : 0);
	v = i2c_smbus_read_byte_data(fd, 0x07);	
	vbat->vdd1_setpoint = (v&0x80 ? (v & 0x3f) * 0.05 + 0.80 : 0);
	v = i2c_smbus_read_byte_data(fd, 0x08);
	vbat->vdd2_setpoint = (v&0x80 ? (v & 0x3f) * 0.05 + 0.80 : 0);
	v = i2c_smbus_read_byte_data(fd, 0x09);
	vbat->vdd3_setpoint = (v&0x80 ? (v & 0x3f) * 0.05 + 2.70 : 0);
	v = i2c_smbus_read_byte_data(fd, 0x0a);
	vbat->vdd4_setpoint = (v&0x80 ? (v & 0x3f) * 0.05 + 2.70 : 0);
	//printf("Setpoints %f %f %f %f\n",vbat->vdd0_setpoint,vbat->vdd1_setpoint,vbat->vdd2_setpoint,vbat->vdd3_setpoint);

	return 0;
}

void vbat_read(vbat_struct *vbat)
{
	vbat->vbat=vbat_get(0);
	vbat->vdd0=vbat_get(4);
	vbat->vdd1=vbat_get(5);
	vbat->vdd2=vbat_get(6);
	vbat->vdd3=vbat_get(7);
	vbat->vdd4=vbat_get(8);
}
