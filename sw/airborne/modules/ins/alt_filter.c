/*
 * Copyright (C) 2010 ENAC
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

#include "modules/ins/alt_filter.h"
#include "modules/gps/gps.h"
#include "modules/sensors/baro_ets.h"

#include "mcu_periph/uart.h"
#include "pprzlink/messages.h"
#include "modules/datalink/downlink.h"

TypeKalman alt_filter;

/* Kalman parameters */
float SigAltiGPS;
float SigAltiAltimetre;
float MarcheAleaBiaisAltimetre;
float MarcheAleaAccelerometre;

/* Function declaration */
void kalmanInit(TypeKalman *k);
void kalmanEstimation(TypeKalman *k, float accVert);
void kalmanCorrectionGPS(TypeKalman *k, float altitude_gps);
void kalmanCorrectionAltimetre(TypeKalman *k, float altitude_altimetre);

/* last measured values */
float last_gps_alt;
float last_baro_alt;

void alt_filter_init(void)
{
  SigAltiGPS = 5.;
  SigAltiAltimetre = 5.;
  MarcheAleaBiaisAltimetre = 0.1;
  MarcheAleaAccelerometre = 0.5;
  last_gps_alt = 0.;
  last_baro_alt = 0.;

  kalmanInit(&alt_filter);
}

void alt_filter_periodic(void)
{
  // estimation at each step
  kalmanEstimation(&alt_filter, 0.);

  // update on new data
  float ga = (float)gps.hmsl / 1000.;
  if (baro_ets_altitude != last_baro_alt) {
    kalmanCorrectionAltimetre(&alt_filter, baro_ets_altitude);
    last_baro_alt = baro_ets_altitude;
  }
  if (ga != last_gps_alt && GpsFixValid()) {
    kalmanCorrectionGPS(&alt_filter, ga);
    last_gps_alt = ga;
  }

  RunOnceEvery(6, DOWNLINK_SEND_VFF(DefaultChannel, DefaultDevice, &baro_ets_altitude,
                                    &(alt_filter.X[0]), &(alt_filter.X[1]), &(alt_filter.X[2]),
                                    &(alt_filter.P[0][0]), &(alt_filter.P[1][1]), &(alt_filter.P[2][2])));

}


/*************************************************************************
 *
 * Filter Initialisation
 *
 *************************************************************************/


void kalmanInit(TypeKalman *k)
{

  k->W[0][0] = MarcheAleaAccelerometre * MarcheAleaAccelerometre; k->W[0][1] = 0;
  k->W[1][0] = 0; k->W[1][1] = MarcheAleaBiaisAltimetre * MarcheAleaBiaisAltimetre;

  k->X[0] = 0;
  k->X[1] = 0;
  k->X[2] = 0;

  k->P[0][0] = 1; k->P[0][1] = 0; k->P[0][2] = 0;
  k->P[1][0] = 0; k->P[1][1] = 1; k->P[1][2] = 0;
  k->P[2][0] = 0; k->P[2][1] = 0; k->P[2][2] = 0.0001;

  k->Te = (1. / 60.);

  // System dynamic
  k->Ad[0][0] = 1; k->Ad[0][1] = k->Te; k->Ad[0][2] = 0;
  k->Ad[1][0] = 0; k->Ad[1][1] = 1; k->Ad[1][2] = 0;
  k->Ad[2][0] = 0; k->Ad[2][1] = 0; k->Ad[2][2] = 1;

  // System command
  k->Bd[0] = pow(k->Te, 2) / 2;
  k->Bd[1] = k->Te;
  k->Bd[2] = 0;

  k->Md[0][0] = pow(k->Te, 1.5) / 2; k->Md[0][1] = 0;
  k->Md[1][0] = pow(k->Te, 0.5); k->Md[1][1] = 0;
  k->Md[2][0] = 0; k->Md[2][1] = pow(k->Te, 0.5);
}


/*************************************************************************
 *
 * Estimation
 *
 *************************************************************************/


void kalmanEstimation(TypeKalman *k, float accVert)
{


  int i, j;
  float I[3][3]; // matrices temporaires
  float J[3][2];

  // Calcul de X
  // X(k+1) = Ad*X(k) + Bd*U(k)
  k->X[0] = k->Ad[0][0] * k->X[0] + k->Ad[0][1] * k->X[1] + k->Ad[0][2] * k->X[2] + k->Bd[0] * accVert;
  k->X[1] = k->Ad[1][0] * k->X[0] + k->Ad[1][1] * k->X[1] + k->Ad[1][2] * k->X[2] + k->Bd[1] * accVert;
  k->X[2] = k->Ad[2][0] * k->X[0] + k->Ad[2][1] * k->X[1] + k->Ad[2][2] * k->X[2] + k->Bd[2] * accVert;

  // Calcul de P
  // P(k+1) = Ad*P(k)*Ad' + Md*W*Md'
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      I[i][j] = k->Ad[i][0] * k->P[0][j] + k->Ad[i][1] * k->P[1][j] + k->Ad[i][2] * k->P[2][j];
    }
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      k->P[i][j] = I[i][0] * k->Ad[j][0] + I[i][1] * k->Ad[j][1] + I[i][2] * k->Ad[j][2];
    }
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 2; j++) {
      J[i][j] = k->Md[i][0] * k->W[0][j] + k->Md[i][1] * k->W[1][j];
    }
  }

  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      k->P[i][j] = k->P[i][j] + J[i][0] * k->Md[j][0] + J[i][1] * k->Md[j][1];
    }
  }

}

/*************************************************************************
 *
 * Correction GPS
 *
 *************************************************************************/

void kalmanCorrectionGPS(TypeKalman *k,
                         float altitude_gps)  // altitude_gps est l'altitude telle qu'elle est mesurée par le GPS
{

  int i, j, div;
  float I[3][3]; // matrice temporaire

  float Kf[3] = { 0., 0., 0. };

  // calcul de Kf
  // C = [1 0 0]
  // div = C*P*C' + R
  div = k->P[0][0] + SigAltiGPS * SigAltiGPS;

  if (fabs(div) > 1e-5) {
    // Kf = P*C'*inv(div)
    Kf[0] = k->P[0][0] / div;
    Kf[1] = k->P[1][0] / div;
    Kf[2] = k->P[2][0] / div;

    // calcul de X
    // X = X + Kf*(meas - C*X)
    float constante = k->X[0];
    for (i = 0; i < 3; i++) {
      k->X[i] = k->X[i] + Kf[i] * (altitude_gps - constante);
    }

    // calcul de P
    // P = P - Kf*C*P
    I[0][0] = Kf[0]; I[0][1] = 0; I[0][2] = 0;
    I[1][0] = Kf[1]; I[1][1] = 0; I[1][2] = 0;
    I[2][0] = Kf[2]; I[2][1] = 0; I[2][2] = 0;

    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
        k->P[i][j] = k->P[i][j] - I[i][0] * k->P[0][j] - I[i][1] * k->P[1][j] - I[i][2] * k->P[2][j];
      }
    }
  }

}

/*************************************************************************
 *
 * Correction altimètre
 *
 *************************************************************************/

void kalmanCorrectionAltimetre(TypeKalman *k, float altitude_altimetre)
{

  int i, j, div;
  float I[3][3]; // matrice temporaire

  float Kf[3] = { 0., 0., 0. };

  // calcul de Kf
  // C = [1 0 1]
  // div = C*P*C' + R
  div = k->P[0][0] + k->P[2][0] + k->P[0][2] + k->P[2][2] + SigAltiAltimetre * SigAltiAltimetre;

  if (fabs(div) > 1e-5) {
    // Kf = P*C'*inv(div)
    Kf[0] = (k->P[0][0] + k->P[0][2]) / div;
    Kf[1] = (k->P[1][0] + k->P[1][2]) / div;
    Kf[2] = (k->P[2][0] + k->P[2][2]) / div;

    // calcul de X
    // X = X + Kf*(meas - C*X)
    float constante = k->X[0] + k->X[2];
    for (i = 0; i < 3; i++) {
      k->X[i] = k->X[i] + Kf[i] * (altitude_altimetre - constante);
    }

    // calcul de P
    // P = P - Kf*C*P
    I[0][0] = Kf[0]; I[0][1] = 0; I[0][2] = Kf[0];
    I[1][0] = Kf[1]; I[1][1] = 0; I[1][2] = Kf[1];
    I[2][0] = Kf[2]; I[2][1] = 0; I[2][2] = Kf[2];

    for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
        k->P[i][j] = k->P[i][j] - I[i][0] * k->P[0][j] - I[i][1] * k->P[1][j] - I[i][2] * k->P[2][j];
      }
    }
  }

}
