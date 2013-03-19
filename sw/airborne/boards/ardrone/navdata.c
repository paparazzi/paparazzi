/*
 * navdata.c
 *
 *  Created on: Oct 18, 2012
 *      Author: dhensen
 */
#include <stdio.h>		// for standard input output
#include <stdlib.h>
#include <fcntl.h> 		// for O_RDWR, O_NOCTTY, O_NONBLOCK
#include <termios.h> 	// for baud rates and options
#include <unistd.h>
#include <string.h>
#include <math.h>
#include "navdata.h"

int nav_fd;
//short int navdata_writeToBlock;

int navdata_init()
{
	port = malloc(sizeof(navdata_port));

	nav_fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (nav_fd == -1)
	{
		perror("navdata_init: Unable to open /dev/ttyO1 - ");
		return 1;
	} else {
		port->isOpen = 1;
	}

	fcntl(nav_fd, F_SETFL, 0); //read calls are non blocking
	//set port options
	struct termios options;
	//Get the current options for the port
	tcgetattr(nav_fd, &options);
	//Set the baud rates to 460800
	cfsetispeed(&options, B460800);
	cfsetospeed(&options, B460800);

	options.c_cflag |= (CLOCAL | CREAD); //Enable the receiver and set local mode
	options.c_iflag = 0; //clear input options
	options.c_lflag = 0; //clear local options
	options.c_oflag &= ~OPOST; //clear output options (raw output)

	//Set the new options for the port
	tcsetattr(nav_fd, TCSANOW, &options);

	// stop acquisition
	uint8_t cmd=0x02;
	write(nav_fd, &cmd, 1);

	// start acquisition
	cmd=0x01;
	write(nav_fd, &cmd, 1);

	navdata = malloc(sizeof(measures_t));
	navdata_imu_available = 0;
	navdata_baro_available = 0;

	port->bytesRead = 0;
	port->totalBytesRead = 0;
	port->packetsRead = 0;
	port->isInitialized = 1;

	previousUltrasoundHeight = 0;

	return 0;
}

void navdata_close()
{
	port->isOpen = 0;
	close(nav_fd);
}

void navdata_read()
{
	int newbytes = 0;

	if (port->isInitialized != 1)
		navdata_init();

	if (port->isOpen != 1)
		return;

	newbytes = read(nav_fd, port->buffer+port->bytesRead, NAVDATA_BUFFER_SIZE-port->bytesRead);

	// because non-blocking read returns -1 when no bytes available
	if (newbytes > 0)
	{
		port->bytesRead += newbytes;
		port->totalBytesRead += newbytes;

//		printf("NewBytes: %d\n", newbytes);
//		printf("BytesRead: %d\n", port->bytesRead);
//		printf("TotalBytesRead: %d\n", port->totalBytesRead);
	}

}

void navdata_update()
{
	navdata_read();

	// while there is something interesting to do...
	while (port->bytesRead >= 60)
	{
		if (port->buffer[0] == NAVDATA_START_BYTE)
		{
			// if checksum is OK
			if ( 1 ) // we dont know how to calculate the checksum
//			if ( navdata_checksum() == 0 )
			{
				memcpy(navdata, port->buffer, NAVDATA_PACKET_SIZE);
				navdata_imu_available = 1;
				navdata_baro_available = 1;
				port->packetsRead++;
//				printf("CCRC=%d, GCRC=%d, error=%d\n", crc, navdata->chksum, abs(crc-navdata->chksum));
				navdata_getHeight();
			}
			navdata_CropBuffer(60);
		}
		else
		{
			// find start byte, copy all data from startbyte to buffer origin, update bytesread
			uint8_t * pint;
			pint = memchr(port->buffer, NAVDATA_START_BYTE, port->bytesRead);

			if (pint != NULL) {
				port->bytesRead -= pint - port->buffer;
				navdata_CropBuffer(pint - port->buffer);
			} else {
				// if the start byte was not found, it means there is junk in the buffer
				port->bytesRead = 0;
			}
		}
	}
}

void navdata_CropBuffer(int cropsize)
{
	if (port->bytesRead - cropsize < 0) {
		printf("BytesRead - Cropsize may not be below zero...");
		return;
	}

	memmove(port->buffer, port->buffer+cropsize, NAVDATA_BUFFER_SIZE-cropsize);
	port->bytesRead -= cropsize;
}

void navdata_event(void (* _navdata_handler)(void)) {
	navdata_update();
	_navdata_handler();
}

int16_t navdata_getHeight() {

	if (navdata->ultrasound > 10000) {
		return previousUltrasoundHeight;
	}

	int16_t ultrasoundHeight = 0;

	ultrasoundHeight = (navdata->ultrasound - 880) / 26.553;
//	ultrasoundHeight /= 100;

//	printf("%d\n", ultrasoundHeight);

	previousUltrasoundHeight = ultrasoundHeight;

	return ultrasoundHeight;
}

uint16_t navdata_checksum() {
	navdata_cks = 0;
	navdata_cks += navdata->nu_trame;
	navdata_cks += navdata->ax;
	navdata_cks += navdata->ay;
	navdata_cks += navdata->az;
	navdata_cks += navdata->vx;
	navdata_cks += navdata->vy;
	navdata_cks += navdata->vz;
	navdata_cks += navdata->temperature_acc;
	navdata_cks += navdata->temperature_gyro;
	navdata_cks += navdata->ultrasound;
	navdata_cks += navdata->us_debut_echo;
	navdata_cks += navdata->us_fin_echo;
	navdata_cks += navdata->us_association_echo;
	navdata_cks += navdata->us_distance_echo;
	navdata_cks += navdata->us_curve_time;
	navdata_cks += navdata->us_curve_value;
	navdata_cks += navdata->us_curve_ref;
	navdata_cks += navdata->nb_echo;
	navdata_cks += navdata->sum_echo;
	navdata_cks += navdata->gradient;
	navdata_cks += navdata->flag_echo_ini;
	navdata_cks += navdata->pressure;
	navdata_cks += navdata->temperature_pressure;
	navdata_cks += navdata->mx;
	navdata_cks += navdata->my;
	navdata_cks += navdata->mz;
//	navdata_cks += navdata->chksum;

	return 0; // we dont know how to calculate the checksum
//	return abs(navdata->chksum - navdata_cks);
}
