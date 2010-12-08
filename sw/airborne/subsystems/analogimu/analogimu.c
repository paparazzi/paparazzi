/*
 * $Id: analogimu.c $
 *  
 * Copyright (C) 2010 Oliver Riesener, Christoph Niemann
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

/** \file analogimu.c
 *  \brief Analog IMU Routines
 *
 */
#if ! (defined SITL || defined HITL)

#include "led.h"
#include "mcu_periph/adc.h"
#include "mcu_periph/uart.h"
//#include "downlink.h"
#include "estimator.h"
//#include "ap_downlink.h"
#include "sys_time.h"

#include "dcm.h"

#include "analogimu_util.h"

#include "analogimu.h"

#endif

#define NB_ADC 8
#define ADC_NB_SAMPLES 16

// variables

uint16_t analog_imu_offset[NB_ADC] = {0,};

static struct adc_buf buf_adc[NB_ADC];
int adc_average[16] = { 0 };

float imu_roll_neutral = RadOfDeg(IMU_ROLL_NEUTRAL_DEFAULT);
float imu_pitch_neutral = RadOfDeg(IMU_PITCH_NEUTRAL_DEFAULT);

#if ! (defined SITL || defined HITL)

// functions
/**
 * accel2ms2():
 *
 * \return accel[ACC_X], accel[ACC_Y], accel[ACC_Z]  
 */
void accel2ms2( void ) {
  accel[ACC_X] = (float)(adc_average[ADC_ACCX])/10.19;
  accel[ACC_Y] = (float)(-adc_average[ADC_ACCY])/10.5;
  accel[ACC_Z] = (float)(adc_average[ADC_ACCZ])/10.4;//chni: needs to be adjusted for earth gravity
}
/**
 * gyro2rads():
 *
 * \return gyro[G_ROLL], gyro[G_PITCH], gyro[G_YAW] 
 */
void gyro2rads( void ) {
  /** 150 grad/sec 10Bit, 3,3Volt, 1rad = 2Pi/1024 => Pi/512 */
  gyro[G_ROLL]  = (float)(adc_average[ADC_ROLL]) / 61.3588;
  gyro[G_PITCH] = (float)(adc_average[ADC_PITCH]) / 57.96;
  gyro[G_YAW]   = (float)(-adc_average[ADC_YAW]) / 60.1;
}

void analog_imu_init( void ) { 
  adc_buf_channel(ADC_CHANNEL_IMU_GROLL, &buf_adc[0], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IMU_GPITCH, &buf_adc[1], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IMU_GYAW, &buf_adc[2], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IMU_ACCX, &buf_adc[5], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IMU_ACCY, &buf_adc[6], ADC_NB_SAMPLES);
  adc_buf_channel(ADC_CHANNEL_IMU_ACCZ, &buf_adc[7], ADC_NB_SAMPLES);
  
#if NB_ADC != 8
#error "8 ADCs expected !"
#endif
  
}

void analog_imu_offset_set( void ) {
  uint8_t i;
  for(i = 0; i < NB_ADC - 1; i++) {
    analog_raw[i] = buf_adc[i].sum / ADC_NB_SAMPLES;
    analog_imu_offset[i] = analog_raw[i];
  }
  analog_imu_offset[7] = analog_raw[7] + 528;// 553; // + Zero of z-acc (without gravity) needs to be adjusted
}
/**
 * analog_imu_update():
 */
void analog_imu_update( void ) {  
  uint8_t i;
  for(i = 0; i < NB_ADC; i++) {
    analog_raw[i] = buf_adc[i].sum / ADC_NB_SAMPLES;
  }
  adc_average[ADC_ROLL]   = analog_raw[0] - analog_imu_offset[0];
  adc_average[ADC_PITCH]  = analog_raw[1] - analog_imu_offset[1];
  adc_average[ADC_YAW]    = analog_raw[2] - analog_imu_offset[2];
  adc_average[ADC_ACCX] = analog_raw[5] - analog_imu_offset[5];
  adc_average[ADC_ACCY] = analog_raw[6] - analog_imu_offset[6];
  adc_average[ADC_ACCZ] = analog_raw[7] - analog_imu_offset[7];
  accel2ms2();
  gyro2rads();
}

/** earth accelecation */
volatile float g = 0.;

// functions

void analog_imu_downlink( void ) {  
  //uint8_t id = 0;
  //float time = GET_CUR_TIME_FLOAT();
  //time *= 1000;//secs to msecs
  //int mx = 0;
  //int my = 0;
  //int mz = 0;
  //DOWNLINK_SEND_HB_FILTER( DefaultChannel,&time, &accel[ACC_X],&accel[ACC_Y],&accel[ACC_Z],&gyro[G_ROLL],&gyro[G_PITCH],&gyro[G_YAW],&heading,&mx,&my,&mz,&euler[EULER_ROLL],&euler[EULER_PITCH],&euler[EULER_YAW], &imu_roll_neutral, &imu_pitch_neutral );
}


/**
 *  matrix transpose is safe to call with c == a if m = n
 *
 * \param [out] *c pointer to destination array 
 * \param [in] *a pointer to source array
 * \param [in] m dimention rows 
 * \param [in] n dimention cols
 * \return result is in *c, or error if a==c and m != n
 */
void matrix_transpose(float *c, float *a, short m, short n)
{
    if ( a != c ) { // quick version
        short i,j;
        // [i][j] ... [m][n]
        for ( i=0 ; i<m ; i++ ) {
            for( j=0 ; j<n ; j++ ) {
                // c[j][i] = a[i][j];
                *(c+(m*j)+i) = *(a+(n*i)+j);
            }
        }
    } else if ( m == n) { // save version
        short i,j;
        // [i][j] ... [m][n]
        for ( i=0 ; i<m ; i++ ) {
            for( j=0 ; j<n ; j++ ) {
                /* c[j][i] = a[i][j]; */
                float vc = *(c+(m*j)+i);
                float va = *(a+(n*i)+j);
                *(c+(m*j)+i) = va;
                *(a+(n*i)+j) = vc;
            }
        }
    } else {
        // error illegal !!
    }
}

/**
 * Minimalistic version to get angles from acceleration
 *
 * \todo why has this function 3 callers ?
 * \return g, angle[ANG_ROLL], angle[ANG_PITCH] 
 */
void accel2euler( void ) {
    // Calculate g ( ||g_vec|| )
    g = sqrt(accel[ACC_X] * accel[ACC_X] +
             accel[ACC_Y] * accel[ACC_Y] +
             accel[ACC_Z] * accel[ACC_Z]);
    if( g < 0.01 && g > -0.01 )
    {
      g=0.01;
    }else{
      //nothing
    }
    //values in radians
#define NEW
#ifdef OLD
    angle[ANG_PITCH] = -asinf( accel[ACC_X] / g );
    angle[ANG_ROLL] = asinf( accel[ACC_Y] / g );
    angle[ANG_YAW] = 0.0;
#endif

#ifdef NEW
  
  float a1 = accel[ACC_X] / -g;
  
  if(a1 > 1.0 && a1 >= 0.0){ 
      a1 = 1.0;
    } else if(a1 < -1.0 && a1 < 0.0){
      a1 = -1.0;
    }
  
  angle[ANG_PITCH] = asinf( a1 );
  
  if(accel[ACC_Z] < 0 && angle[ANG_PITCH] > 0) angle[ANG_PITCH] = + 3.145/2 + (3.145/2 - angle[ANG_PITCH]);
  else if (accel[ACC_Z] < 0 && angle[ANG_PITCH] < 0) angle[ANG_PITCH] =  -3.145/2 - (3.145/2 + angle[ANG_PITCH]);
  
  
  if( accel[ACC_Z] < 0.01 && accel[ACC_Z] > -0.01 )
  {
      accel[ACC_Z]=0.01;
  }else{
      //nothing
  }
  
  
  angle[ANG_ROLL] = atan2f( accel[ACC_Y], accel[ACC_Z] );
  //angle[ANG_PITCH] = -atan2f( accel[ACC_X] , accel[ACC_Z] );
  angle[ANG_YAW] = 0.0;
#endif
}


void estimator_update_state_analog_imu( void ) {
#undef ANGLE_FROM_ACCEL
#ifdef ANGLE_FROM_ACCEL
  estimator_phi = (float)(atan2f((float)((analog_raw[6]-510)),(float)(-(analog_raw[7]-510))));
  estimator_theta = (float)(atan2f((float)(-(analog_raw[5]-510)),(float)(-(analog_raw[7]-510))));
#else

  analog_imu_update();
  
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();

  // return euler angles to phi and theta
  estimator_phi = euler[EULER_ROLL]-imu_roll_neutral;
  //estimator_phi = angle[ANG_ROLL];
  estimator_theta = euler[EULER_PITCH]-imu_pitch_neutral;
  //estimator_theta = angle[ANG_PITCH];
  estimator_psi = euler[EULER_YAW];

#endif
}
#endif
