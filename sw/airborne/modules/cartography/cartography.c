/*
 * Copyright (C) 2011  Vandeportaele Bertrand
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
/** \file cartography.c
 *  \brief Navigation functions for cartography of the ground
 *
 */




#include "state.h"
#include "stdio.h"

#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"

#include "std.h"  //macros pas mal dans sw/include

////////////////////////////////////////////////////////////////////////////////////////////////
//for fast debbuging, the simulation can be accelerated using the gaia software from an xterm console
//              /home/bvdp/paparazzi3/sw/simulator/gaia
////////////////////////////////////////////////////////////////////////////////////////////////
// for explanations about debugging macros:
//http://gcc.gnu.org/onlinedocs/cpp/Stringification.html#Stringification

// Be carefull not to use printf function in ap compilation, only use it in sim compilation
// the DEBUG_PRINTF should be defined only in the sim part of the makefile airframe file
#ifdef DEBUG_PRINTF
int CPTDEBUG = 0;
#define PRTDEB(TYPE,EXP) \
  printf("%5d: " #EXP ": %"#TYPE"\n",CPTDEBUG,EXP);fflush(stdout);CPTDEBUG++;
#define PRTDEBSTR(EXP) \
  printf("%5d: STR: "#EXP"\n",CPTDEBUG);fflush(stdout);CPTDEBUG++;
#else
#define PRTDEB(TYPE,EXP) \
  ;

#define PRTDEBSTR(EXP) \
  ;
#endif

/*
 exemple of use for theese macros
 PRTDEBSTR(Init polysurvey)
 PRTDEB(u,SurveySize)

 PRTDEB(lf,PolygonCenter.x)
 PRTDEB(lf,PolygonCenter.y)
 */
////////////////////////////////////////////////////////////////////////////////////////////////

#define DISTXY(P1X,P1Y,P2X,P2Y)\
  (sqrt(  ( (P2X-P1X) * (P2X-P1X) ) + ( (P2Y-P1Y) * (P2Y-P1Y) )  ) )

#define NORMXY(P1X,P1Y)\
  (sqrt(  ( (P1X) * (P1X) ) + ( (P1Y) * (P1Y) )  ) )


//max distance between the estimator position and an objective points to consider that the objective points is atteined

#define DISTLIMIT 30

////////////////////////////////////////////////////////////////////////////////////////////////

uint16_t railnumberSinceBoot =
  1; //used to count the number of rails the plane has achieved since the boot, to number the sequences of saved images
//the number 1 is reserved for snapshot fonctions that take only one image, the 2-65535 numbers are used to number the following sequences

////////////////////////////////////////////////////////////////////////////////////////////////
#define USE_ONBOARD_CAMERA 1

#if USE_ONBOARD_CAMERA
uint16_t camera_snapshot_image_number = 0;
#endif




////////////////////////////////////////////////////////////////////////////////////////////////
bool survey_losange_uturn;//this flag indicates if the aircraft is turning between 2 rails (1) or if it is flying straight forward in the rail direction (0)

int railnumber;  //indicate the number of the rail being acquired
int numberofrailtodo;

float distrail;  //distance between adjacent rails in meters, the value is set in the init function
float distplus;  //distance that the aircraft should travel before and after a rail before turning to the next rails in meters, the value is set in the init function

float distrailinteractif =
  60; //a cheangable value that can be set interactively in the GCS, not used at that time, it can be used to choose the right value while the aircraft is flying


static struct point point1, point2, point3; // 3 2D points used for navigation
static struct point pointA, pointB, pointC; // 3 2D points used for navigation
static struct point vec12, vec13;
float norm12, norm13; // norms of 12 & 13  vectors




float tempx, tempy; //temporary points for exchanges
float angle1213; //angle between 12 & 13  vectors
float signforturn; //can be +1 or -1, it is used to compute the direction of the UTURN between rails

float tempcircleradius;// used to compute the radius of the UTURN after a rail
float circleradiusmin = 40;

////////////////////////////////////////////////////////////////////////////////////////////////
float normBM, normAM, distancefromrail;


float course_next_rail;
float angle_between;

bool ProjectionInsideLimitOfRail;




#include "modules/cartography/cartography.h"
#include "generated/modules.h"

#include "subsystems/datalink/downlink.h"
#include "mcu_periph/uart.h"
#include "std.h"


////////////////////////////////////////////////////////////////////////////////////////////////

void init_carto(void)
{
}

void periodic_downlink_carto(void)
{
  DOWNLINK_SEND_CAMERA_SNAPSHOT(DefaultChannel, DefaultDevice, &camera_snapshot_image_number);
}

void start_carto(void)
{
}

void stop_carto(void)
{
}



///////////////////////////////////////////////////////////////////////////////////////////////
bool nav_survey_Inc_railnumberSinceBoot(void)
{
  railnumberSinceBoot++;
  return false;
}
///////////////////////////////////////////////////////////////////////////////////////////////
bool nav_survey_Snapshoot(void)
{
  camera_snapshot_image_number = railnumberSinceBoot;
  PRTDEBSTR(SNAPSHOT)
  cartography_periodic_downlink_carto_status = MODULES_START;
  return false;

}
///////////////////////////////////////////////////////////////////////////////////////////////
bool nav_survey_Snapshoot_Continu(void)
{
  camera_snapshot_image_number = railnumberSinceBoot;
  PRTDEBSTR(SNAPSHOT)
  cartography_periodic_downlink_carto_status = MODULES_START;
  return true;

}
///////////////////////////////////////////////////////////////////////////////////////////////
bool nav_survey_StopSnapshoot(void)
{
  camera_snapshot_image_number = 0;
  PRTDEBSTR(STOP SNAPSHOT)
  cartography_periodic_downlink_carto_status = MODULES_START;
  return false;

}
///////////////////////////////////////////////////////////////////////////////////////////////

bool nav_survey_computefourth_corner(uint8_t wp1, uint8_t wp2,  uint8_t wp3, uint8_t wp4)
{
  waypoints[wp4].x = waypoints[wp2].x + waypoints[wp3].x - waypoints[wp1].x;
  waypoints[wp4].y = waypoints[wp2].y + waypoints[wp3].y - waypoints[wp1].y;

  PRTDEBSTR(nav_survey_computefourth_corner)
  PRTDEB(f, waypoints[wp4].x)
  PRTDEB(f, waypoints[wp4].y)
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////
bool  nav_survey_ComputeProjectionOnLine(struct point pointAf, struct point pointBf, float pos_xf, float pos_yf,
    float *normAMf, float *normBMf, float *distancefromrailf);


bool  nav_survey_ComputeProjectionOnLine(struct point pointAf, struct point pointBf, float pos_xf, float pos_yf,
    float *normAMf, float *normBMf, float *distancefromrailf)
//return if the projection of the estimator on the AB line is located inside the AB interval
{
  float a, b, c, xa, xb, xc, ya, yb, yc;
  float f;
  float AA1;
  float BB1;
  float YP;
  float XP;

  float AMx, AMy, BMx, BMy;
  //+++++++++++++++++++++++++ATTENTION AUX DIVISIONS PAR 0!!!!!!!!!!!!!!!



  xb = pointAf.x;
  yb = pointAf.y;

  xc = pointBf.x;
  yc = pointBf.y;

  xa = pos_xf;
  ya = pos_yf;

  //calcul des parametres de la droite pointAf pointBf
  a = yc - yb;
  b = xb - xc;
  c = (yb - yc) * xb + (xc - xb) * yb ;

  //calcul de la distance de la droite à l'avion


  if (fabs(a) > 1e-10) {
    *distancefromrailf = fabs((a * xa + b * ya + c) / sqrt(a * a + b *
                              b));  //denominateur =0 iniquement si a=b=0 //peut arriver si 2 waypoints sont confondus
  } else {
    return 0;
  }

  PRTDEB(f, a)
  PRTDEB(f, b)
  PRTDEB(f, c)
  PRTDEB(f, *distancefromrailf)


  // calcul des coordonnées du projeté orthogonal M(xx,y) de A sur (BC)
  AA1 = (xc - xb);
  BB1 = (yc - yb);
  if (fabs(AA1) > 1e-10) {
    f = (b - (a * BB1 / AA1));
    if (fabs(f) > 1e-10) {
      YP = (-(a * xa) - (a * BB1 * ya / AA1) - c) / f;
    } else {
      return 0;
    }
  } else {
    return 0;
  }




  XP = (-c - b * YP) / a ; //a !=0 deja testé avant
  //+++++++++++++++++++++++++ATTENTION AUX DIVISIONS PAR 0!!!!!!!!!!!!!!!
  //+++++++++++++++++++++++++ATTENTION AUX DIVISIONS PAR 0!!!!!!!!!!!!!!!
  //+++++++++++++++++++++++++ATTENTION AUX DIVISIONS PAR 0!!!!!!!!!!!!!!!

  PRTDEB(f, AA1)
  PRTDEB(f, BB1)
  PRTDEB(f, YP)
  PRTDEB(f, XP)

  AMx = XP - pointAf.x;
  AMy = YP - pointAf.y;
  BMx = XP - pointBf.x;
  BMy = YP - pointBf.y;

  *normAMf = NORMXY(AMx, AMy);
  *normBMf = NORMXY(BMx, BMy);

  PRTDEB(f, *normAMf)
  PRTDEB(f, *normBMf)

  if (((*normAMf) + (*normBMf)) > 1.05 * DISTXY(pointBf.x, pointBf.y, pointAf.x, pointAf.y)) {
    PRTDEBSTR(NOT INSIDE)
    return 0;
  } else {
    PRTDEBSTR(INSIDE)
    return 1;
  }
}
///////////////////////////////////////////////////////////////////////////
//if distrailinit = 0, the aircraft travel from  wp1 -> wp2 then do the inverse travel passing through  the wp3,
//This mode could be use to register bands of images aquired in a first nav_survey_losange_carto, done perpendicularly
bool nav_survey_losange_carto_init(uint8_t wp1, uint8_t wp2,  uint8_t wp3, float distrailinit, float distplusinit)
{
  //PRTDEBSTR(nav_survey_losange_carto_init)
  survey_losange_uturn = false;


  point1.x = waypoints[wp1].x; //the coordinates are in meter units, taken from the flight plan, in float type
  point1.y = waypoints[wp1].y;
  point2.x = waypoints[wp2].x;
  point2.y = waypoints[wp2].y;
  point3.x = waypoints[wp3].x;
  point3.y = waypoints[wp3].y;

  PRTDEB(u, wp1)
  PRTDEB(f, point1.x)
  PRTDEB(f, point1.y)

  PRTDEB(u, wp2)
  PRTDEB(f, point2.x)
  PRTDEB(f, point2.y)

  PRTDEB(u, wp3)
  PRTDEB(f, point3.x)
  PRTDEB(f, point3.y)



  vec12.x = point2.x - point1.x;
  vec12.y = point2.y - point1.y;
  PRTDEB(f, vec12.x)
  PRTDEB(f, vec12.y)

  //TODO gerer le cas ou un golio met les points à la meme position -> norm=0 > /0
  norm12 = NORMXY(vec12.x, vec12.y);

  PRTDEB(f, norm12)


  vec13.x = point3.x - point1.x;
  vec13.y = point3.y - point1.y;
  PRTDEB(f, vec13.x)
  PRTDEB(f, vec13.y)

  norm13 = NORMXY(vec13.x, vec13.y);
  PRTDEB(f, norm13)

  //if (distrail<1e-15)  //inutile distrail=0 pour recollage et dans ce cas, il prend la valeur norm13
  //  return false;


  if (fabs(distrailinit) <= 1) {
    //is distrailinit==0, then the aircraft should do 2 passes to register the bands
    distrail = norm13;
    numberofrailtodo = 1;
  } else {
    //no, so normal trajectory
    distrail = fabs(distrailinit);
    numberofrailtodo = ceil(norm13 / distrail); //round to the upper integer
  }

  distplus = fabs(distplusinit);


  PRTDEB(f, distrail)
  PRTDEB(f, distplus)
  PRTDEB(d, numberofrailtodo)
  PRTDEB(d, railnumber)
  PRTDEB(d, railnumberSinceBoot)

  railnumber = -1; // the state is before the first rail, which is numbered 0

  if (norm12 < 1e-15) {
    return false;
  }
  if (norm13 < 1e-15) {
    return false;
  }


  angle1213 = (180 / 3.14159) * acos((((vec12.x * vec13.x) + (vec12.y * vec13.y))) / (norm12 *
                                     norm13)); //oriented angle between 12 and 13 vectors

  angle1213 = atan2f(vec13.y, vec13.x) - atan2f(vec12.y, vec12.x);
  while (angle1213 >= M_PI) { angle1213 -= 2 * M_PI; }
  while (angle1213 <= -M_PI) { angle1213 += 2 * M_PI; }

  PRTDEB(f, angle1213)

  if (angle1213 > 0) {
    signforturn = -1;
  } else {
    signforturn = 1;
  }


  return false; //Init function must return false, so that the next function in the flight plan is automatically executed
  //dans le flight_plan.h
  //        if (! (nav_survey_losange_carto()))
  //          NextStageAndBreak();
}
////////////////////////////////////////////////////////////////////////////////////////////////
bool nav_survey_losange_carto(void)
{
  //test pour modifier en vol la valeur distrail

  //distrail=distrailinteractif;


  //by default, a 0 is sent in the message DOWNLINK_SEND_CAMERA_SNAPSHOT,
  //if the aircraft is inside the region to map, camera_snapshot_image_number will be equal to the number of rail since the last boot (not since the nav_survey_losange_carto_init, in order to get different values for differents calls to the cartography function  (this number is used to name the images on the hard drive
  camera_snapshot_image_number = 0;


  PRTDEB(f, distrail)



  PRTDEBSTR(nav_survey_losange_carto)
  PRTDEB(d, railnumber)

  PRTDEB(d, railnumberSinceBoot)

  //PRTDEB(f,stateGetPositionEnu_f()->x)
  //PRTDEB(f,stateGetPositionEnu_f()->y)

  //sortir du bloc si données abhérantes
  if (norm13 < 1e-15) {
    PRTDEBSTR(norm13 < 1e-15)
    return false;
  }
  if (norm12 < 1e-15) {
    PRTDEBSTR(norm13 < 1e-15)
    return false;
  }
  if (distrail < 1e-15) {
    PRTDEBSTR(distrail < 1e-15)
    return false;
  }

  if (survey_losange_uturn == FALSE) {

    if (railnumber == -1) {
      //se diriger vers le début du 1°rail
      PRTDEBSTR(approche debut rail 0)
      pointA.x = point1.x - (vec12.x / norm12) * distplus * 1.2; //on prend une marge plus grande pour arriver en ce point
      pointA.y = point1.y - (vec12.y / norm12) * distplus * 1.2; //car le virage n'est pas tres bien géré


      pointB.x = point2.x + (vec12.x / norm12) * distplus * 1.2; //on prend une marge plus grande pour arriver en ce point
      pointB.y = point2.y + (vec12.y / norm12) * distplus * 1.2; //car le virage n'est pas tres bien géré

      //PRTDEB(f,pointA.x)
      //PRTDEB(f,pointA.y)


      //the following test can cause problem when the aircraft is quite close to the entry point, as it can turn around for infinte time
      //if ( DISTXY(stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y,pointA.x,pointA.y)  >DISTLIMIT)
      //if ( DISTXY(stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y,pointA.x,pointA.y)  >DISTLIMIT)


      nav_survey_ComputeProjectionOnLine(pointA, pointB, stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, &normAM,
                                         &normBM, &distancefromrail);

      if ((DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointA.x, pointA.y)  > 2 * DISTLIMIT)
          || (normBM < (DISTXY(pointB.x, pointB.y, pointA.x, pointA.y)))) {
        nav_route_xy(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointA.x, pointA.y);
        //nav_route_xy(pointB.x, pointB.y,pointA.x,pointA.y);
      } else {
        PRTDEBSTR(debut rail 0)
        //un fois arrivé, on commence le 1° rail;
        railnumber = 0;
        railnumberSinceBoot++;

      }
    }


    if (railnumber >= 0) {
      pointA.x = (point1.x - ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
      pointA.y = (point1.y - ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);

      pointB.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
      pointB.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);





      if ((railnumber % 2) == 0) { //rail n0, 2, 4, donc premiere direction, de wp1 vers wp2
        //rien a faire
      } else { //if ((railnumber %2)==1) //rail n1, 3, 5, donc seconde direction, de wp2 vers wp1
        //echange pointA et B
        tempx = pointA.x;
        tempy = pointA.y;
        pointA.x = pointB.x;
        pointA.y = pointB.y;
        pointB.x = tempx;
        pointB.y = tempy;

      }

      //  PRTDEB(f,pointA.x)
      //  PRTDEB(f,pointA.y)
      //  PRTDEB(f,pointB.x)
      //  PRTDEB(f,pointB.y)
      ProjectionInsideLimitOfRail = nav_survey_ComputeProjectionOnLine(pointA, pointB, stateGetPositionEnu_f()->x,
                                    stateGetPositionEnu_f()->y, &normAM, &normBM, &distancefromrail);



      //  if ( ( DISTXY(stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y,pointB.x,pointB.y)  >DISTLIMIT)  &&
      //    (normBM>(DISTXY(pointB.x,pointB.y,pointA.x,pointA.y))))


      if (!((DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointB.x, pointB.y) < DISTLIMIT)
            || (ProjectionInsideLimitOfRail && (normAM > (DISTXY(pointB.x, pointB.y, pointA.x, pointA.y))))))
        //    (normBM>(DISTXY(pointB.x,pointB.y,pointA.x,pointA.y))))
      {
        nav_route_xy(pointA.x, pointA.y, pointB.x, pointB.y);
        PRTDEBSTR(NAVROUTE)


        //est ce que l'avion est dans la zone ou il doit prendre des images?
        //DEJA APPELE AVANT LE IF
        //  nav_survey_ComputeProjectionOnLine(pointA,pointB,stateGetPositionEnu_f()->x,stateGetPositionEnu_f()->y,&normAM,&normBM,&distancefromrail);

        if ((normAM > distplus) && (normBM > distplus) && (distancefromrail < distrail / 2)) {
          //CAMERA_SNAPSHOT_REQUIERED=true;
          //camera_snapshot_image_number++;
          camera_snapshot_image_number = railnumberSinceBoot;
          PRTDEBSTR(SNAPSHOT)
        }

      }

      else { // virage
        //PRTDEBSTR(debut rail suivant)
        railnumber++;
        railnumberSinceBoot++;

        PRTDEB(d, railnumber)
        PRTDEB(d, railnumberSinceBoot)
        //CAMERA_SNAPSHOT_REQUIERED=true;
        //camera_snapshot_image_number++;

        PRTDEBSTR(UTURN)
        survey_losange_uturn = true;

      }

      if (railnumber > numberofrailtodo) {
        PRTDEBSTR(fin nav_survey_losange_carto)
        return false; //apparament, l'avion va au bloc suivant lorsque la fonction renvoie false
      }

    }
  } else { // (survey_losange_uturn==TRUE)


    if (distrail < 200) {
      //tourne autour d'un point à mi chemin entre les 2 rails

      //attention railnumber a été incrémenté  en fin du rail précédent

      if ((railnumber % 2) == 1) { //rail précédent n0, 2, 4, donc premiere direction, de wp1 vers wp2
        PRTDEBSTR(UTURN - IMPAIR)
        //fin du rail précédent
        pointA.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber - 1) * (vec13.x / norm13) * distrail);
        pointA.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber - 1) * (vec13.y / norm13) * distrail);
        //début du rail suivant
        pointB.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointB.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);
        //milieu
        waypoints[0].x = (pointA.x + pointB.x) / 2;
        waypoints[0].y = (pointA.y + pointB.y) / 2;

        tempcircleradius = distrail / 2;
        if (tempcircleradius < circleradiusmin) {
          tempcircleradius = circleradiusmin;
        }


        //fin du rail suivant
        pointC.x = (point1.x + ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointC.y = (point1.y + ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);


        course_next_rail = atan2(pointC.x - pointB.x, pointC.y - pointB.y);
        PRTDEB(f, course_next_rail)
        PRTDEB(f, stateGetHorizontalSpeedDir_f())

        angle_between = (course_next_rail - stateGetHorizontalSpeedDir_f());
        while (angle_between > M_PI) { angle_between -= 2 * M_PI; }
        while (angle_between < -M_PI) { angle_between += 2 * M_PI; }

        angle_between = DegOfRad(angle_between);
        PRTDEB(f, angle_between)
        //if (angle_between> -10 && angle_between< 10)   PRTDEBSTR(ON SE CASSE)

        NavCircleWaypoint(0, signforturn * tempcircleradius);
        if ((DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointB.x, pointB.y)  < DISTLIMIT)
            || (angle_between > -10 && angle_between < 10)) {
          //l'avion fait le rail suivant
          survey_losange_uturn = false;
          PRTDEBSTR(FIN UTURN - IMPAIR)
        }
      } else { //if ((railnumber %2)==0) //rail précédent n1, 3, 5, donc seconde direction, de wp2 vers wp1
        PRTDEBSTR(UTURN - PAIR)
        //fin du rail précédent
        pointA.x = (point1.x - ((vec12.x / norm12) * distplus)) + ((railnumber - 1) * (vec13.x / norm13) * distrail);
        pointA.y = (point1.y - ((vec12.y / norm12) * distplus)) + ((railnumber - 1) * (vec13.y / norm13) * distrail);
        //début du rail suivant
        pointB.x = (point1.x - ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointB.y = (point1.y - ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);
        //milieu
        waypoints[0].x = (pointA.x + pointB.x) / 2;
        waypoints[0].y = (pointA.y + pointB.y) / 2;

        tempcircleradius = distrail / 2;
        if (tempcircleradius < circleradiusmin) {
          tempcircleradius = circleradiusmin;
        }




        //fin du rail suivant
        pointC.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointC.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);


        course_next_rail = atan2(pointC.x - pointB.x, pointC.y - pointB.y);
        PRTDEB(f, course_next_rail)
        PRTDEB(f, stateGetHorizontalSpeedDir_f())

        angle_between = (course_next_rail - stateGetHorizontalSpeedDir_f());
        while (angle_between > M_PI) { angle_between -= 2 * M_PI; }
        while (angle_between < -M_PI) { angle_between += 2 * M_PI; }

        angle_between = DegOfRad(angle_between);
        PRTDEB(f, angle_between)
        //if (angle_between> -10 && angle_between< 10)   PRTDEBSTR(ON SE CASSE)

        NavCircleWaypoint(0, signforturn * (-1)*tempcircleradius);
        if ((DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointB.x, pointB.y)  < DISTLIMIT)
            || (angle_between > -10 && angle_between < 10)) {
          //l'avion fait le rail suivant
          survey_losange_uturn = false;
          PRTDEBSTR(FIN UTURN - PAIR)
        }
      }
    } else {
      //Le virage serait trop grand, on va en ligne droite pour ne pas trop éloigner l'avion

      if ((railnumber % 2) == 1) { //rail précédent n0, 2, 4, donc premiere direction, de wp1 vers wp2
        PRTDEBSTR(TRANSIT - IMPAIR)
        //fin du rail précédent
        pointA.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber - 1) * (vec13.x / norm13) * distrail);
        pointA.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber - 1) * (vec13.y / norm13) * distrail);
        //début du rail suivant
        pointB.x = (point2.x + ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointB.y = (point2.y + ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);
        nav_route_xy(pointA.x, pointA.y, pointB.x, pointB.y);
        if (DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointB.x, pointB.y)  < DISTLIMIT) {
          //l'avion fait le rail suivant
          survey_losange_uturn = false;
          PRTDEBSTR(FIN TRANSIT - IMPAIR)
        }
      } else { //if ((railnumber %2)==0) //rail précédent n1, 3, 5, donc seconde direction, de wp2 vers wp1
        PRTDEBSTR(TRANSIT - PAIR)
        //fin du rail précédent
        pointA.x = (point1.x - ((vec12.x / norm12) * distplus)) + ((railnumber - 1) * (vec13.x / norm13) * distrail);
        pointA.y = (point1.y - ((vec12.y / norm12) * distplus)) + ((railnumber - 1) * (vec13.y / norm13) * distrail);
        //début du rail suivant
        pointB.x = (point1.x - ((vec12.x / norm12) * distplus)) + ((railnumber) * (vec13.x / norm13) * distrail);
        pointB.y = (point1.y - ((vec12.y / norm12) * distplus)) + ((railnumber) * (vec13.y / norm13) * distrail);
        nav_route_xy(pointA.x, pointA.y, pointB.x, pointB.y);
        if (DISTXY(stateGetPositionEnu_f()->x, stateGetPositionEnu_f()->y, pointB.x, pointB.y)  < DISTLIMIT) {
          //l'avion fait le rail suivant
          survey_losange_uturn = false;
          PRTDEBSTR(FIN TRANSIT - PAIR)
        }

      }

    }
  }


  //////////////// FAUT IL METTRE l'APPEL A CES FONCTIONS???????????????,
  //NavVerticalAutoThrottleMode(0.); /* No pitch */
  //NavVerticalAltitudeMode(WaypointAlt(wp1), 0.); /* No preclimb */



  cartography_periodic_downlink_carto_status = MODULES_START;

  return true; //apparament pour les fonctions de tache=> true
}
////////////////////////////////////////////////////////////////////////////////////////////////


