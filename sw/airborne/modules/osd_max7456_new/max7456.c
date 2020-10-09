/*
 * Copyright (C) 2013 Chris
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
/*  MAX 7456 OSD DRIVER
*   Date 21 Aug 2013
*   VERSION 1.1
*   The driver now checks the busy flag and it can justify the characters in left, right or center.
*/ 
#include "std.h"
#include "stdio.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/spi.h"
#include "led.h"
#include "autopilot.h"

#include "subsystems/navigation/common_nav.h"
#include "firmwares/fixedwing/nav.h"
#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "subsystems/datalink/datalink.h"
#include "subsystems/electrical.h"

//#include "messages.h"
#include "subsystems/datalink/downlink.h"
#include "state.h"

// Peripherials
#include "max7456.h"

#if !defined(SITL)

#if defined(STM32) || defined(BOARD_KROOZ)

#if !defined(USE_SPI2)
#define USE_SPI2			1
#endif
#if !defined(MAX7456_SPI_DEV)
#define MAX7456_SPI_DEV 		spi2
#endif
#if !defined(MAX7456_SLAVE_IDX)
#define USE_SPI_SLAVE1			1
#define MAX7456_SLAVE_IDX		SPI_SLAVE1
#endif


#elif defined(BOARD_REVO_MINI)

#if !defined(USE_SPI3)
#define USE_SPI3			1
#endif
#if !defined(MAX7456_SPI_DEV)
#define MAX7456_SPI_DEV 		spi3
#endif
#if !defined(MAX7456_SLAVE_IDX)
#define USE_SPI_SLAVE0			1
#define MAX7456_SLAVE_IDX		SPI_SLAVE0
#endif

#elif defined(BOARD_OLIMEX_F4)

#if !defined(USE_SPI2)
#define USE_SPI2			1
#endif
#if !defined(MAX7456_SPI_DEV)
#define MAX7456_SPI_DEV 		spi2
#endif
#if !defined(MAX7456_SLAVE_IDX)
#define USE_SPI_SLAVE1			1
#define MAX7456_SLAVE_IDX		SPI_SLAVE1
#endif

#elif defined(LPC21)

#if !defined(USE_SPI1)
#define USE_SPI1			1
#endif
#if !defined(MAX7456_SPI_DEV)
#define MAX7456_SPI_DEV 		spi1
#endif
#if !defined(MAX7456_SLAVE_IDX)
#define USE_SPI_SLAVE0			1
#define MAX7456_SLAVE_IDX		SPI_SLAVE0
#endif

#elif defined(BOARD_MATEK_F405_WING)

#if !defined(USE_SPI2)
#define USE_SPI2			1
#endif
#if !defined(MAX7456_SPI_DEV)
#define MAX7456_SPI_DEV 		spi2
#endif
#if !defined(MAX7456_SLAVE_IDX)
#define USE_SPI_SLAVE1			1
#define MAX7456_SLAVE_IDX		SPI_SLAVE1
#endif

#if !defined(USE_PAL_FOR_OSD_VIDEO)
#define USE_PAL_FOR_OSD_VIDEO 0
#endif

#endif

//OSD REGISTER ADDRESSES
#define OSD_VM0_REG			0x00
#define OSD_VM1_REG			0x01
#define OSD_DMM_REG			0x04
#define OSD_DMAH_REG			0x05
#define OSD_DMAL_REG			0x06
#define OSD_DMDI_REG			0x07
#define OSD_OSDBL_REG			0x6C
#define OSD_OSDBL_REG_R			0xEC
#define OSD_STAT_REG			0xA0

//OSD BIT POSITIONS
#define OSD_VIDEO_MODE_PAL		(1<<6)          // Default = NTSC 
#define OSD_SYNC_INTERNAL		((1<<5)|(1<<4)) // Default = AUTO
#define OSD_SYNC_EXTERNAL		((1<<5)		// Default = AUTO
#define OSD_IMAGE_ENABLE		(1<<3)		// Default = OSD OFF
#define OSD_REFRESH_ON_NEXT_VSYNC	(1<<2)		// Default = immediately refresh video
#define OSD_RESET			(1<<1)		// VM0 reg, hardware set to 0 after reset 
#define OSD_VOUT_DISABLE		(1<<0)		// default= VIDEO OUT ENABLED
#define OSD_8BIT_MODE			(1<<6)		// default= 16 BIT MODE
#define OSD_BLINK_CHAR			(1<<4)		// default= No BLINKING
#define OSD_INVERT_PIXELS		(1<<3)		// default= No INVERSION
#define OSD_CLEAR_DISPLAY_MEMORY	(1<<2)		// DMM reg, default = 0	
#define OSD_AUTO_INCREMENT_MODE		(1<<0)	        // default = NO AUTO INCREMENT 

#define OSD_PAL_DETECTED		(1<<0)
#define OSD_NTSC_DETECTED		(1<<1)

#define OSD_STRING_SIZE			31
#define osd_sprintf			_osd_sprintf

/******************************************************************************************/
/*************************   PRIVATE FUNCTION PROTOTYPES    *******************************/
/******************************************************************************************/
static char ascii_to_osd_c( char c);
static void osd_put_s(char *string,  uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column);
static bool _osd_sprintf(char* buffer, char* string, float value);
static void check_osd_status(void);
static void draw_osd_flying_screen(void);

#ifdef BOARD_KROOZ
static void max7456_before_cb(struct spi_transaction *trans);
static void max7456_after_cb(struct spi_transaction *trans);
#endif

#if OSD_USE_MAG_COMPASS && !defined(SITL)
float MAG_Heading = 0;
#endif



/******************************************************************************************/
/*******************************   GLOBAL VARIABLES   *************************************/
/******************************************************************************************/
struct spi_transaction max7456_trans;

uint8_t osd_spi_tx_buffer[2];
uint8_t osd_spi_rx_buffer[2];
char osd_string[OSD_STRING_SIZE];
char osd_char = ' ';
uint8_t step = 0;
uint16_t osd_char_address = 0;
uint8_t  osd_attr = FALSE;

enum max7456_osd_status_codes{ 
     OSD_UNINIT,
     OSD_INIT1,
     OSD_INIT2,
     OSD_INIT3,
     OSD_INIT4,
     OSD_READ_STATUS,
     OSD_IDLE,
     OSD_S_STEP1,
     OSD_S_STEP2,
     OSD_S_STEP3,
     OSD_FINISHED,
};

enum osd_attributes{
     BLINK = OSD_BLINK_CHAR,
     INVERT = OSD_INVERT_PIXELS,
     L_JUST = 0x00,
     R_JUST = 0x01,
     C_JUST = 0x02,

};        

                   

uint8_t max7456_osd_status = OSD_UNINIT;
uint8_t osd_enable = TRUE;
uint8_t osd_enable_val = OSD_IMAGE_ENABLE;
uint8_t osd_stat_reg = 0;
bool  osd_stat_reg_valid = FALSE;


/******************************************************************************************/
/*************************   PUBLIC FUNCTION DEFINITIONS   ********************************/
/******************************************************************************************/
//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111
void max7456_init(void) {

#ifdef BOARD_KROOZ
#define SPI_SELECT_SLAVE_OSD_PORT GPIOB
#define SPI_SELECT_SLAVE_OSD_PIN GPIO2

gpio_setup_output(SPI_SELECT_SLAVE_OSD_PORT, SPI_SELECT_SLAVE_OSD_PIN);
gpio_set(SPI_SELECT_SLAVE_OSD_PORT, SPI_SELECT_SLAVE_OSD_PIN);
#endif

#ifdef BOARD_KROOZ
max7456_trans.slave_idx     = 0;              //Krooz uses PB2 as an OSD slave CS so we will
max7456_trans.select        = SPINoSelect;       //Select/Unselect it through call back functions.
#else
max7456_trans.slave_idx     = MAX7456_SLAVE_IDX;
max7456_trans.select        = SPISelectUnselect;
#endif
max7456_trans.cpol          = SPICpolIdleLow;
max7456_trans.cpha          = SPICphaEdge1;
max7456_trans.dss           = SPIDss8bit;
max7456_trans.bitorder      = SPIMSBFirst;
max7456_trans.cdiv          = SPIDiv64;
max7456_trans.output_length = sizeof(osd_spi_tx_buffer);
max7456_trans.output_buf    = (uint8_t*) osd_spi_tx_buffer;
max7456_trans.input_length  = 0;
max7456_trans.input_buf     = (uint8_t*)osd_spi_rx_buffer;
#ifdef BOARD_KROOZ
max7456_trans.before_cb     = max7456_before_cb; //Selects the OSD in KROOZ board (PB2)
max7456_trans.after_cb      = max7456_after_cb;  //Unselects the OSD in KROOZ board (PB2)
#else
max7456_trans.before_cb     = NULL;
max7456_trans.after_cb      = NULL;
#endif

osd_enable = 1;
osd_enable_val = OSD_IMAGE_ENABLE;
max7456_osd_status = OSD_UNINIT;
step = 0;

return;
}

//222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222
void max7456_periodic(void){

//This code is executed always and checks if the "osd_enable" var has been changed by telemetry.
//If yes then it commands a reset but this time turns on or off the osd overlay, not the video. 
if (max7456_osd_status == OSD_IDLE){
   if(osd_enable > 1){ osd_enable = 1; }
   if ((osd_enable<<3) !=  osd_enable_val){
      osd_enable_val = (osd_enable<<3);
      max7456_osd_status = OSD_UNINIT;
   }
}

//INITIALIZATION OF THE OSD
if (max7456_osd_status == OSD_UNINIT){
   step = 0;
   max7456_trans.status = SPITransDone;
   max7456_trans.output_buf[0] = OSD_VM0_REG;
   //This operation needs at least 100us but when the periodic function will be invoked again
   //sufficient time will have elapsed even with at a periodic frequency of 1000 Hz 
   max7456_trans.output_buf[1] = OSD_RESET; 
   max7456_osd_status = OSD_INIT1;
   spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
}
else
if (max7456_osd_status == OSD_INIT2){
   max7456_trans.output_length = 1;
   max7456_trans.input_length = 1;
   max7456_trans.output_buf[0] = OSD_OSDBL_REG_R;
   max7456_osd_status = OSD_INIT3;
   spi_submit(&(MAX7456_SPI_DEV), &max7456_trans); 
}
else
if (max7456_osd_status == OSD_IDLE && osd_enable > 0){ // DRAW THE OSD SCREEN
   
   draw_osd_flying_screen();

} // if (max7456_osd_status == OSD_IDLE && osd_enable > 0)

#if OSD_USE_MAG_COMPASS && !defined(SITL)

#ifndef AHRS_MAG_DECLINATION
#define AHRS_MAG_DECLINATION 5.3 //Sept 2020 for Greece
#endif
float mag_heading_x = 0;
float mag_heading_y = 0;

MAG_Heading = 0;


int32_t x = 0, y = 0, z = 0;

x = (imu.mag_unscaled.x - imu.mag_neutral.x)*IMU_MAG_X_SIGN;
y = (imu.mag_unscaled.y - imu.mag_neutral.y)*IMU_MAG_Y_SIGN;
z = (imu.mag_unscaled.z - imu.mag_neutral.z)*IMU_MAG_Z_SIGN;

float cos_roll; float sin_roll; float cos_pitch; float sin_pitch;
struct FloatEulers* att = stateGetNedToBodyEulers_f();
cos_roll = cosf(att->phi);
sin_roll = sinf(att->phi);
cos_pitch = cosf(att->theta);
sin_pitch = sinf(att->theta);
// Pitch&Roll Compensation:
mag_heading_x = x*cos_pitch + y*sin_roll*sin_pitch + z*cos_roll*sin_pitch;
mag_heading_y = y*cos_roll - z*sin_roll;
// Magnetic Heading
MAG_Heading = atan2(-mag_heading_y,mag_heading_x);
//compass = MAG_Heading;
// MAG_Heading = atan2(imu.mag.y, -imu.mag.x); // Magnetic Heading 2D
// Declination correction (if supplied)
if ( AHRS_MAG_DECLINATION != 0.0 ){
   MAG_Heading = MAG_Heading + AHRS_MAG_DECLINATION;
   if (MAG_Heading > M_PI){  // Angle normalization (-180 deg to 180 deg)
      MAG_Heading -= (2.0 * M_PI);

   }else if (MAG_Heading < -M_PI){ MAG_Heading += (2.0 * M_PI); }
}
#endif
return;
}

//333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333
void max7456_event(void){ 

static uint8_t x = 0;

if (max7456_trans.status == SPITransSuccess) {
   max7456_trans.status = SPITransDone;

   switch (max7456_osd_status){

          case (OSD_INIT1):
              max7456_osd_status = OSD_INIT2;
          break;

          case (OSD_INIT3):
               max7456_trans.input_length = 0; 
               max7456_trans.output_length = 2;
               max7456_trans.output_buf[0] = OSD_OSDBL_REG;
               max7456_trans.output_buf[1] = max7456_trans.input_buf[0] & (~(1<<4));
               max7456_osd_status = OSD_INIT4;
               spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  
          break;

          case (OSD_INIT4):
               max7456_trans.output_buf[0] = OSD_VM0_REG;
#if USE_PAL_FOR_OSD_VIDEO
               max7456_trans.output_buf[1] = OSD_VIDEO_MODE_PAL|osd_enable_val;
#else
//#warning OSD USES NTSC
               max7456_trans.output_buf[1] = osd_enable_val;
#endif
               max7456_osd_status = OSD_FINISHED;
               spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  
          break;

          case (OSD_S_STEP1):
               max7456_trans.input_length = 0;
               max7456_trans.output_length = 2;
               max7456_trans.output_buf[0] = OSD_DMAL_REG;
               max7456_trans.output_buf[1] = (uint8_t)(osd_char_address);
               max7456_osd_status = OSD_S_STEP2;
               spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
          break;
          
          case (OSD_S_STEP2): 
               max7456_trans.output_length = 2;
               max7456_trans.output_buf[0] = OSD_DMM_REG;
               max7456_trans.output_buf[1] = OSD_AUTO_INCREMENT_MODE | osd_attr;
               max7456_osd_status = OSD_S_STEP3;
               spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
               x = 0;
          break;
          case (OSD_S_STEP3):  
               max7456_trans.output_length = 1; //1 byte tranfers, auto address incrementing.
               if (osd_string[x] != 0XFF){
                  max7456_trans.output_buf[0] = osd_string[x++];
                  spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
               }else{ 
                       max7456_trans.output_buf[0] = 0xFF; //Exit the auto increment mode
                       max7456_osd_status = OSD_FINISHED;
                       spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
                    }
          break;  

          case (OSD_READ_STATUS):
               osd_stat_reg = max7456_trans.input_buf[0];
               osd_stat_reg_valid = TRUE;
               max7456_trans.status = SPITransDone;
               max7456_osd_status = OSD_IDLE;
          break;

          case (OSD_FINISHED):
               max7456_trans.status = SPITransDone;
               max7456_osd_status = OSD_IDLE;
          break;

          default: break;  
 
   } // switch (max7456_osd_status)

} // if (max7456_trans.status == SPITransSuccess)

return;
}

/******************************************************************************************/
/***********************    PRIVATE FUNCTION DEFINITIONS    *******************************/
/******************************************************************************************/
/*1111111111111111111111111111111111111111111111111111111111111111111111111111111111111111*/
static void draw_osd_flying_screen(void){

float temp = 0;

//uint8_t pitch = 0;
struct FloatEulers* att = stateGetNedToBodyEulers_f();
struct EnuCoor_f* pos = stateGetPositionEnu_f();

float ph_x = waypoints[WP_HOME].x - pos->x;
float ph_y = waypoints[WP_HOME].y - pos->y;

// osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)
//attributes =  BLINK, INVERT, C_JUST (Center Justified), R_JUST (Right Justified)
// L_JUST (Left Justified) which is also the default behavior, same as attribute FALSE or 0
//Attributes can be ORed example "C_JUST|BLINK|INVERT"
//char_nb = the number of chars to reserve (clear) 
// row = OSD ROW NUMBER
// column = OSD COLUMN NUMBER
// I HAVE TRIED TO USE ALL DIFFERENT COMBINATIONS IN THE BELOW CODE.
//************************************ P A L *********************************//
//************************************ P A L *********************************//
//************************************ P A L *********************************//
#if USE_PAL_FOR_OSD_VIDEO
#pragma message "OSD USES PAL"


   switch (step){

          case (0):
               osd_put_s("HDG", FALSE, 3, 1, 14);
               step = 1;
          break;
          case (1):
               osd_put_s("DISTANCE", FALSE, 8, 13, 12);
               step = 10;
          break; 

          case (10):
#if CAMERA_MODULE_WITH_LOCK_AVAILABLE
#pragma message "OSD MESSAGE: camera lock available"
               if (cam_lock) osd_put_s("( )", BLINK, 3, 7, 14); else
#endif
	       osd_put_s("( )", FALSE, 3, 7, 14);  
               step = 20;
          break;

          case (20):
               temp = ((float)electrical.vsupply)/10;
               osd_sprintf(osd_string, "%.1fV", temp );
               if (temp > LOW_BAT_LEVEL){
                  osd_put_s(osd_string, L_JUST, 5, 1, 1);

               }else{ osd_put_s(osd_string, L_JUST|BLINK|INVERT, 5, 1, 1); }
               step = 30;
          break;

          case (30):
#if OSD_USE_MAG_COMPASS && !defined(SITL)
#pragma message "OSD USES THE MAGNETIC HEADING"
               temp = DegOfRad(MAG_Heading);
               if (temp < 0){ temp += 360; }
#else
#pragma message "OSD USES THE GPS HEADING"
               temp = DegOfRad(state.h_speed_dir_f);
               if (temp < 0){ temp += 360; } 
#endif
               osd_sprintf(osd_string, "%.0f", temp);
               osd_put_s(osd_string, C_JUST, 3, 2, 15);
               step = 40;
          break;

          case (40):
               osd_sprintf(osd_string, "%.0f Km", (state.h_speed_norm_f*3.6));
               osd_put_s(osd_string, R_JUST, 6, 1, 29);
               step = 41; 
          break;
          case (41):
               osd_sprintf(osd_string, "%.0fThr", (((float)ap_state->commands[COMMAND_THROTTLE]/MAX_PPRZ)*100));
               osd_put_s(osd_string, R_JUST, 5, 2, 29);
               step = 50;
          break;

          case (50):
#if OSD_USE_BARO_ALTITUDE && !defined(SITL)
#pragma message "OSD ALTITUDE IS COMING FROM BAROMETER"
#if defined BARO_ALTITUDE_VAR
               osd_sprintf(osd_string, "%.0fm", BARO_ALTITUDE_VAR );
#else
#warning OSD USES THE DEFAULT BARO ALTITUDE VARIABLE
               osd_sprintf(osd_string, "%.0fm", baro_alt );
#endif
#else
#pragma message "ALTITUDE IS COMING FROM GPS"
               osd_sprintf(osd_string, "%.0fm", GetPosAlt() );
#endif
               osd_put_s(osd_string, L_JUST, 6, 14, 1); // "FALSE = L_JUST
               step = 60;
          break;

          case (60):
               osd_sprintf(osd_string, "%.0fm", (float)(sqrt(ph_x*ph_x + ph_y *ph_y)));
               osd_put_s(osd_string, C_JUST, 6, 14, 16);
               step = 70;
          break; 

          case (70):
               osd_sprintf(osd_string, "%.1fVZ", stateGetSpeedEnu_f()->z);
               osd_put_s(osd_string, R_JUST, 9, 14, 29);
               step = 80;
          break;
          // A Text PFD as graphics are not the strong point of the MAX7456 
          // In order to level the aircraft while fpving 
          // just move the stick to the opposite direction from the angles shown on the osd
          // and that's why positive pitch (UP) is shown below the OSD center
          case (80):
               if(DegOfRad(att->theta) > 2){ 
                 osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
                 osd_put_s(osd_string, C_JUST, 5, 5, 15);
 
               }else{ osd_put_s("    ", C_JUST, 5, 5, 15); }
               step = 90;
          break;

          case (90):
               if(DegOfRad(att->theta) < -2){ 
                 osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
                 osd_put_s(osd_string, C_JUST, 5, 9, 15);
 
               }else{ osd_put_s("   ", C_JUST, 5, 9, 15); }
               step = 100;
          break;

          case (100):
               if(DegOfRad(att->phi) > 2){ 
                 osd_sprintf(osd_string, "%.0f>", DegOfRad(att->phi));
                 osd_put_s(osd_string, FALSE, 5, 7, 18);
 
               }else{ osd_put_s("     ", FALSE, 5, 7, 18); }
               step = 110;
          break;

          case (110):
               if(DegOfRad(att->phi) < -2){ 
                 osd_sprintf(osd_string, "<%.0f", DegOfRad(fabs(att->phi)) );
                 osd_put_s(osd_string, R_JUST, 5, 7, 13);
 
               }else{ osd_put_s("     ", R_JUST, 5, 7, 13); }
               step = 120;
          break;

          case (120):
               check_osd_status();
               step = 121;
          break;
#if defined(SONAR_SCALE)
#pragma message "SONAR ALTITUDE AVAILABLE"
          case (121):
               if (sonar_adc.distance < 8.0){
                  osd_put_s("PULL UP", C_JUST|BLINK|INVERT, 7, 3, 15); 

               }else{ osd_put_s("         ", FALSE, 7, 3, 15); }
            
               step = 122;
          break;
       
          case (122):
               if (sonar_adc.distance < 8.0){
                  osd_sprintf(osd_string, "S%.2fm", sonar_adc.distance );
                  osd_put_s(osd_string, C_JUST, 6, 13, 1); 

               }else{ osd_put_s("        ", FALSE, 6, 4, 15); }
            
               step = 10;
          break;
#endif
          


          default:	step = 10; break;
   }

//************************************ N T S C *********************************//
//************************************ N T S C *********************************//
//************************************ N T S C *********************************//
#else // IF NTSC IS USED WE HAVE ONLY 13 ROWS NOT 15
#pragma message "OSD USES NTSC"


   switch (step){

          case (0):
               osd_put_s("HDG", FALSE, 3, 1, 14);
               step = 1;
          break;
          case (1):
               osd_put_s("DISTANCE", FALSE, 8, 11, 12);
               step = 10;
          break; 

          case (10):
#if CAMERA_MODULE_WITH_LOCK_AVAILABLE
#pragma message "OSD MESSAGE: camera lock available"
               if (cam_lock) osd_put_s("( )", BLINK, 3, 7, 14); else
#endif
	       osd_put_s("( )", FALSE, 3, 7, 14);  
               step = 20;
          break;

          case (20):
               temp = ((float)electrical.vsupply);
               osd_sprintf(osd_string, "%.1fV", temp );
               if (temp > LOW_BAT_LEVEL){
                  osd_put_s(osd_string, L_JUST, 5, 1, 2);

               }else{ osd_put_s(osd_string, L_JUST|BLINK|INVERT, 5, 0, 1); }
               step = 30;
          break;

          case (30):
#if OSD_USE_MAG_COMPASS && !defined(SITL)
#pragma message "OSD USES THE MAGNETIC HEADING"
               temp = DegOfRad(MAG_Heading);
               if (temp < 0){ temp += 360; }
#else
#pragma message "OSD USES THE GPS HEADING"
               temp = DegOfRad(state.h_speed_dir_f);
               if (temp < 0){ temp += 360; } 
#endif
               osd_sprintf(osd_string, "%.0f", temp);
               osd_put_s(osd_string, C_JUST, 3, 2, 15);
               step = 40;
          break;

          case (40):
               osd_sprintf(osd_string, "%.0f KM", (state.h_speed_norm_f*3.6));
               osd_put_s(osd_string, R_JUST, 6, 1, 29);
               step = 41; 
          break;
          case (41):
               osd_sprintf(osd_string, "%.0fTHR", (((float)ap_state->commands[COMMAND_THROTTLE]/MAX_PPRZ)*100));
               osd_put_s(osd_string, R_JUST, 5, 2, 29);
               step = 50;
          break;

          case (50):
#if OSD_USE_BARO_ALTITUDE && !defined(SITL)
#pragma message "OSD ALTITUDE IS COMING FROM BAROMETER"
#if defined BARO_ALTITUDE_VAR
               osd_sprintf(osd_string, "%.0fm", BARO_ALTITUDE_VAR );
#else
#warning OSD USES THE DEFAULT BARO ALTITUDE VARIABLE
               osd_sprintf(osd_string, "%.0fM", baro_alt );
#endif
#else
#pragma message "ALTITUDE IS COMING FROM GPS"
               osd_sprintf(osd_string, "%.0fM", GetPosAlt() );
#endif
               osd_put_s(osd_string, L_JUST, 6, 12, 1); // "FALSE = L_JUST
               step = 60;
          break;

          case (60):
               osd_sprintf(osd_string, "%.0fM", (float)(sqrt(ph_x*ph_x + ph_y *ph_y)));
               osd_put_s(osd_string, C_JUST, 6, 12, 16);
               step = 70;
          break; 

          case (70):
               osd_sprintf(osd_string, "%.1fVZ", stateGetSpeedEnu_f()->z);
               osd_put_s(osd_string, R_JUST, 9, 12, 29);
               step = 80;
          break;
          // A Text PFD as graphics are not the strong point of the MAX7456 
          // In order to level the aircraft while fpving 
          // just move the stick to the opposite direction from the angles shown on the osd
          // and that's why positive pitch (UP) is shown below the OSD center
          case (80):
               if(DegOfRad(att->theta) > 2){ 
                 osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
                 osd_put_s(osd_string, C_JUST, 5, 5, 15);
 
               }else{ osd_put_s("    ", C_JUST, 5, 5, 15); }
               step = 90;
          break;

          case (90):
               if(DegOfRad(att->theta) < -2){ 
                 osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
                 osd_put_s(osd_string, C_JUST, 5, 9, 15);
 
               }else{ osd_put_s("   ", C_JUST, 5, 9, 15); }
               step = 100;
          break;

          case (100):
               if(DegOfRad(att->phi) > 2){ 
                 osd_sprintf(osd_string, "%.0f>", DegOfRad(att->phi));
                 osd_put_s(osd_string, FALSE, 5, 7, 18);
 
               }else{ osd_put_s("     ", FALSE, 5, 7, 18); }
               step = 110;
          break;

          case (110):
               if(DegOfRad(att->phi) < -2){ 
                 osd_sprintf(osd_string, "<%.0f", DegOfRad(fabs(att->phi)) );
                 osd_put_s(osd_string, R_JUST, 5, 7, 13);
 
               }else{ osd_put_s("     ", R_JUST, 5, 7, 13); }
               step = 120;
          break;

          case (120):
               check_osd_status();
               step = 121;
          break;
#if defined(SONAR_SCALE)
#pragma message "SONAR ALTITUDE AVAILABLE"
          case (121):
               if (sonar_adc.distance < 8.0){
                  osd_put_s("PULL UP", C_JUST|BLINK|INVERT, 7, 3, 15); 

               }else{ osd_put_s("         ", FALSE, 7, 3, 15); }
            
               step = 122;
          break;
       
          case (122):
               if (sonar_adc.distance < 8.0){
                  osd_sprintf(osd_string, "S%.2fM", sonar_adc.distance );
                  osd_put_s(osd_string, C_JUST, 6, 11, 1); 

               }else{ osd_put_s("        ", FALSE, 6, 11, 15); }
            
               step = 10;
          break;
#endif
          

/*
          case (122):
               if (max7456_osd_status == OSD_IDLE && osd_stat_reg_valid == TRUE){
                  if ( (osd_stat_reg & (OSD_PAL_DETECTED|OSD_NTSC_DETECTED)) == 0 ){ 
                     osd_put_s("NO VIDEO", L_JUST|BLINK|INVERT, 8, 3, 1); 

                  }else{ osd_put_s("           ", FALSE, 8, 3, 1); }
               }
               step = 10;
          break;
*/
          default:	step = 10; break;
   }
#endif


return;
}

/*22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222*/
#ifdef BOARD_KROOZ

static void max7456_before_cb(struct spi_transaction *trans __attribute__ ((unused))){ 

gpio_clear(SPI_SELECT_SLAVE_OSD_PORT, SPI_SELECT_SLAVE_OSD_PIN);



return;
}

/*33333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333*/
static void max7456_after_cb(struct spi_transaction *trans __attribute__ ((unused))){ 

gpio_set(SPI_SELECT_SLAVE_OSD_PORT, SPI_SELECT_SLAVE_OSD_PIN);



return;
}
#endif

/*44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444*/
static char ascii_to_osd_c(char c){

return(c);

if (c >= '0' && c <= '9'){
   if (c == '0'){ c -= 38; }else{  c -= 48; }
}
else
if (c >= 'A' && c <= 'Z'){ 
    c -= 54; 
}
else
if (c >= 'a' && c <= 'z'){ 
   c -= 60; 

}else{
       switch (c){
              case('('): c = 0x3f; break;
              case(')'): c = 0x40; break;
              case('.'): c = 0x41; break;
              case('?'): c = 0x42; break;
              case(';'): c = 0x43; break;
              case(':'): c = 0x44; break;
              case(','): c = 0x45; break;
              //case('''): c = 0x46; break;
              case('/'): c = 0x47; break;
              case('"'): c = 0x48; break;
              case('-'): c = 0x49; break;
              case('<'): c = 0x4A; break;
              case('>'): c = 0x4B; break;
              case('@'): c = 0x4C; break;
              case(' '): c = 0x00; break;
              case('\0'): c = 0xFF; break;
              default  : break;
       }
     }

return(c);
}                       

/*55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555*/
//static void osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)
//attributes =  BLINK, INVERT, C_JUST (Center Justified), R_JUST (Right Justified)
//Attributes can be ORed example BLINK|INVERT
//char_nb = the number of chars to reserve (clear) 
// row = OSD ROW NUMBER
// column = OSD COLUMN NUMBER
static void osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column){

int8_t x = 0, idx = 0, post_offset = 0, aft_offset = 0, string_len = 0;
char osd_buf[OSD_STRING_SIZE];

if (row > 15){ column = 15; }
if (column > 29){ column = 29; }
 
// translate the string and put it to the "osd_string" '\0' = 0xff
x = 0;
while (*(string+x) != '\0'){ osd_string[x] = ascii_to_osd_c(*(string+x)); x++; }
osd_string[x] = 0xff;
string_len = x;
idx = x;

if (attributes & C_JUST){ 
   if(char_nb%2 == 0){ char_nb++; }
   post_offset = (char_nb - string_len)/2;
   aft_offset = char_nb - string_len;
   if( ((int8_t)column-(char_nb/2)) >= 0){ column -= (char_nb/2); } 
   for(x=0; x<30; x++){ osd_buf[x] = 0; } // FILL WITH SPACES
   // COPY THE ORIGINAL STRING TO ITS NEW POSITION
   for(x=0; x<string_len; x++){ osd_buf[post_offset+x] = osd_string[x]; } 
   osd_buf[string_len+aft_offset] = 0xFF; // TERMINATE THE MODIFIED STRING
   // COPY THE MODIFIED STRING TO MAIN OSD STRING
   x = 0;
   do{ osd_string[x] = osd_buf[x]; }while(osd_buf[x++] != 0xFF);
}
else
if (attributes & R_JUST){
   //if(x){ x -= 1; }
   //if (char_nb < string_len){ char_nb = string_len; }
   if( ((int8_t)column-char_nb) >= 0){ column -= char_nb; }
   if (((int8_t)char_nb-string_len) >= 0){ post_offset = char_nb-string_len; }else{post_offset = 0; }
   //ADD LEADING SPACES
   //First shift right the string and then add spaces at the beggining
   while(idx >= 0){ osd_string[idx+post_offset] = osd_string[idx]; idx--; }
   idx = 0;
   while(idx < post_offset){ osd_string[idx] = 0; idx++; } 
   //osd_string[idx] = 0xff;

}else{
        //Adjust for the reserved character number.
        for (x=0; x < (int8_t)(sizeof(osd_string)); x++){ if(osd_string[x] == 0xFF){ break; } }
        for (; x< char_nb; x++){  osd_string[x] = 0; }
        osd_string[x] = 0xff;
     } 

osd_char_address = ((uint16_t)row*30) + column;
osd_attr = (attributes & (BLINK|INVERT)); 
//TRIGGER THE SPI TRANSFERS. The rest of the spi transfers occur in the "max7456_event" function.
if (max7456_osd_status == OSD_IDLE){
    max7456_trans.output_length = 2;
    max7456_trans.output_buf[0] = OSD_DMAH_REG;
    max7456_trans.output_buf[1] = (uint8_t)((osd_char_address>>8) & 0x0001);
    max7456_osd_status = OSD_S_STEP1;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);

}

return;
}


/*66666666666666666666666666666666666666666666666666666666666666666666666666666666666666666666*/
// A VERY VERY STRIPED DOWN sprintf function suitable only for the paparazzi OSD.
static bool _osd_sprintf(char* buffer, char* string, float value){

uint8_t param_start = 0;
uint8_t param_end = 0;
uint8_t frac_nb = 0;
uint8_t digit = 0;
uint8_t x = 0, y = 0, z = 0;

uint16_t i_dec = 0;
uint16_t i_frac = 0;

char to_asc[10] = {48,48,48,48,48,48,48,48,48,48};

// Clear the osd string.
for (x=0; x < sizeof(osd_string); x++){ osd_string[x] = 0; }
x = 0;
// Search for the prameter start and stop positions.
while (*(string+x) != '\0'){
      if (*(string+x) == '%'){
         param_start = x; 

      }else if (*(string+x) == 'f'){ param_end = x; break; }
      x++;
}
// find and bound the precision specified.
frac_nb = *(string+param_end-1) - 48; // Convert to number, ASCII 48 = '0'
if(frac_nb > 3){ frac_nb = 3; }       // Bound value.

y = (sizeof(to_asc)- 1); // Point y to the end of the array.
i_dec = abs((int16_t)value);
// Fist we will deal with the fractional part if specified.
if (frac_nb > 0 && frac_nb <= 3){
   i_frac = abs((int16_t)((value - (int16_t)value) * 1000)); // Max precision is 3 digits.
   x = 100;
   z = frac_nb;
   do{                            // Example if frac_nb=2 then 952 will show as .95
        z--;
        digit = (i_frac / x);
        to_asc[y+z] = digit + 48; // Convert to ASCII
        i_frac -= digit * x;      // Calculate the remainder.
        x /= 10;                  // 952-(9*100) = 52, 52-(10*5)=2 etc.

     }while(z > 0);

   y -= frac_nb;     // set y to point where the dot must be placed.
   to_asc[y] = '.';
   y--;             // Set y to point where the rest of the numbers must be written.
}

// Now it is time for the integer part. "y" already points to the position just before the dot.
do{
     to_asc[y] = (i_dec % 10) + 48;            //Write at least one digit even if value is zero.
     i_dec /= 10;
     if (i_dec <= 0){                          // This way the leading zero is ommited.
        if(value < 0){ y--; to_asc[y] = '-'; } // Place the minus sign if needed.
        break;

     }else{ y--; }

  }while(1); 

// Fill the buffer with the characters in the beggining of the string if any.
for (x=0; x<param_start; x++){
    *(buffer+x) = *(string+x);
}

// x is now pointing to the next character in osd_string. 
// y is already pointing to the first digit or negative sign in "to_asc" array.
while (y < sizeof(to_asc)){
      *(buffer + x) = to_asc[y];
      x++; y++;
}
// x is now pointing to the next character in osd_string. 
// "param_end" is pointing to the last format character in the string.
do{
     param_end++; 
     *(buffer + x++) = *(string+param_end);

  }while(*(string+param_end) != '\0'); //Write the rest of the string including the terminating char.


return(0);
}

static void check_osd_status(void){

osd_stat_reg_valid = FALSE;

if (max7456_osd_status == OSD_IDLE){
   max7456_trans.output_length = 1;
   max7456_trans.input_length = 1;
   max7456_trans.output_buf[0] = OSD_STAT_REG;
   max7456_osd_status = OSD_READ_STATUS;
   spi_submit(&(MAX7456_SPI_DEV), &max7456_trans); 
}   

return;
}

#else	// #if !defined(SITL)

uint8_t osd_enable;

void max7456_init(void){ }
void max7456_periodic(void){ }
void max7456_event(void){ }

#endif

/**********************************************************************************************/
/***********************************   T H E   E N D    ***************************************/
/**********************************************************************************************/

