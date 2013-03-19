/*
 * navdata.h
 *
 *  Created on: Oct 17, 2012
 *      Author: dhensen
 */

#ifndef NAVDATA_H_
#define NAVDATA_H_

#include <stdint.h>

#define NAVDATA_PACKET_SIZE 60
#define NAVDATA_BUFFER_SIZE 80
#define NAVDATA_START_BYTE 0x3a

typedef struct {
	uint8_t isInitialized;
	uint8_t isOpen;
	uint16_t bytesRead;
	uint32_t totalBytesRead;
	uint32_t packetsRead;
	uint8_t buffer[NAVDATA_BUFFER_SIZE];
} navdata_port;

typedef struct
{
  uint16_t taille;
  uint16_t nu_trame;

  uint16_t ax;
  uint16_t ay;
  uint16_t az;

  int16_t vx;
  int16_t vy;
  int16_t vz;
  uint16_t temperature_acc;
  uint16_t temperature_gyro;

  uint16_t ultrasound;

  uint16_t us_debut_echo;
  uint16_t us_fin_echo;
  uint16_t us_association_echo;
  uint16_t us_distance_echo;

  uint16_t us_curve_time;
  uint16_t us_curve_value;
  uint16_t us_curve_ref;

  uint16_t nb_echo;

  uint32_t sum_echo; //unsigned long
  int16_t gradient;

  uint16_t flag_echo_ini;

  int32_t pressure; //long
  int16_t temperature_pressure;

  int16_t mx;
  int16_t my;
  int16_t mz;

  uint16_t chksum;

} __attribute__ ((packed)) measures_t;

measures_t* navdata;
navdata_port* port;
uint16_t navdata_cks;
uint8_t navdata_imu_available;
uint8_t navdata_baro_available;
int16_t previousUltrasoundHeight;

int navdata_init(void);
void navdata_close(void);

void navdata_read(void);
void navdata_update(void);
void navdata_CropBuffer(int cropsize);

uint16_t navdata_checksum(void);

void navdata_event(void (* _navdata_handler)(void));

int16_t navdata_getHeight(void);

#define NavdataEvent(_navdata_handler) { \
  navdata_event(_navdata_handler); \
}

#endif /* NAVDATA_H_ */
