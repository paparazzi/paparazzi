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
 * @file modules/display/max7456.c
 * Maxim MAX7456 single-channel monochrome on-screen display driver.
 *
 */

#include "std.h"
//#include "stdio.h"

#include "modules/core/commands.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/spi.h"

#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "autopilot.h"
#include "modules/energy/electrical.h"
#include "state.h"

// for GetPosAlt, include correct header until we have unified API
#if defined(FIXEDWING_FIRMWARE)
//#include "modules/nav/nav.h"
#include "modules/nav/common_nav.h"
#elif defined(ROTORCRAFT_FIRMWARE)
#include "firmwares/rotorcraft/navigation.h"
#endif
#if DOWNLINK
#include "modules/datalink/telemetry.h"
#endif

// Peripherials
#include "modules/display/max7456.h"
#include "modules/display/max7456_regs.h"

#define OSD_STRING_SIZE     31
#define osd_sprintf         _osd_sprintf

#if !defined(OSD_USE_FLOAT_LOW_PASS_FILTERING)
#define OSD_USE_FLOAT_LOW_PASS_FILTERING
#endif

//LOW PASS filter strength, cannot be 0, MAX=16
#if !defined(AMPS_LOW_PASS_FILTER_STRENGTH) || AMPS_LOW_PASS_FILTER_STRENGTH == 0
#define AMPS_LOW_PASS_FILTER_STRENGTH 6
#endif

#if !defined(SPEED_LOW_PASS_FILTER_STRENGTH) || SPEED_LOW_PASS_FILTER_STRENGTH == 0
#define SPEED_LOW_PASS_FILTER_STRENGTH 6
#endif

#if !defined(BAT_CAPACITY)
#pragma message "BAT_CAPACITY not defined, 5000 mah will be used."
#define BAT_CAPACITY  5000.0
#endif

#if defined(FIXEDWING_FIRMWARE)

#if !defined(LOITER_BAT_CURRENT)
#pragma message "LOITER_BAT_CURRENT not defined, 10 Amps will be used for LOITER current draw."
#define LOITER_BAT_CURRENT  10.0
#endif

#if !defined(STALL_AIRSPEED)
#pragma message "STALL_AIRSPEED not defined, 10 m/s will be used."
#define STALL_AIRSPEED  10.0
#endif

#if !defined(MINIMUM_AIRSPEED)
#pragma message "MINIMUM_AIRSPEED not defined, 1.3 * STALL_SPEED will be used"
#define MINIMUM_AIRSPEED (1.3f * STALL_AIRSPEED)
#endif

#endif

#if !defined(IMU_MAG_X_SIGN)
#define IMU_MAG_X_SIGN 1
#endif
#if !defined(IMU_MAG_X_SIGN)
#define IMU_MAG_Y_SIGN 1
#endif
#if !defined(IMU_MAG_X_SIGN)
#define IMU_MAG_Z_SIGN 1
#endif


typedef struct {
  float fx;
  float fy;
  float fz;
} VECTOR;

typedef struct {
  float fx1; float fx2; float fx3;
  float fy1; float fy2; float fy3;
  float fz1; float fz2; float fz3;
} MATRIX;

#if defined(FIXEDWING_FIRMWARE)
static void mag_compass(void);
#endif
static void send_mag_heading(struct transport_tx *trans, struct link_device *dev);
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC);
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC);
static float home_direction(void);
static char ascii_to_osd_c(char c);
static void calc_flight_time_left(void);
static void draw_osd(void);
static void osd_put_s(char *string,  uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column);
static bool _osd_sprintf(char *buffer, char *string, float value);

struct spi_transaction max7456_trans;

uint8_t osd_spi_tx_buffer[2];
uint8_t osd_spi_rx_buffer[2];
char osd_string[OSD_STRING_SIZE];
char osd_str_buf[OSD_STRING_SIZE];
char osd_char = ' ';
uint8_t step = 0;
uint16_t osd_char_address = 0;
uint8_t  osd_attr = false;

enum max7456_osd_status_codes {
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

enum osd_attributes {
  BLINK = OSD_BLINK_CHAR,
  INVERT = OSD_INVERT_PIXELS,
  L_JUST = 0x00,
  R_JUST = 0x01,
  C_JUST = 0x02,

};

uint8_t max7456_osd_status = OSD_UNINIT;
uint8_t osd_enable = true;
uint8_t osd_enable_val = OSD_IMAGE_ENABLE;
uint8_t osd_stat_reg = 0;
uint32_t max_flyable_distance_left = 0;

float mag_course_deg = 0;
float mag_heading_rad = 0;
float gps_course_deg = 0;
float home_dir_deg = 0;

#if defined(FIXEDWING_FIRMWARE)
// Periodic function called with a frequency defined in the module .xml file
void mag_compass(void)
{

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  float cos_roll; float sin_roll; float cos_pitch; float sin_pitch; float mag_x; float mag_y;
  static float mag_declination = 0;
  static bool declination_calculated = false;

  struct imu_mag_t *mag = imu_get_mag(ABI_BROADCAST, false);
  if(mag == NULL)
    return;

  cos_roll = cosf(att->phi);
  sin_roll = sinf(att->phi);
  cos_pitch = cosf(att->theta);
  sin_pitch = sinf(att->theta);
  // Pitch&Roll Compensation:
  mag_x = mag->scaled.x * cos_pitch + mag->scaled.y * sin_roll * sin_pitch + mag->scaled.z * cos_roll * sin_pitch;
  mag_y = mag->scaled.y * cos_roll - mag->scaled.z * sin_roll;

  // Magnetic Heading N = 0, E = 90, S = +-180, W = -90
  mag_heading_rad = atan2(-mag_y, mag_x);
#if defined(AHRS_MAG_DECLINATION)
  if (AHRS_MAG_DECLINATION != 0.0) {
    //conversion from degrees to radians is done in the airframe.h file
    mag_heading_rad = mag_heading_rad + AHRS_MAG_DECLINATION;
  }
#endif
  if (mag_heading_rad > M_PI) { // Angle normalization (-180 deg to 180 deg)
    mag_heading_rad -= (2.0 * M_PI);
  } else if (mag_heading_rad < -M_PI) { mag_heading_rad += (2.0 * M_PI); }

  if (declination_calculated == false) {
#if defined(NOMINAL_AIRSPEED)
    if (gps.fix == GPS_FIX_3D && stateGetHorizontalSpeedNorm_f() > (float)NOMINAL_AIRSPEED) {
#else
    if (gps.fix == GPS_FIX_3D && stateGetHorizontalSpeedNorm_f() > 10.0) {
#endif
      mag_declination = stateGetHorizontalSpeedDir_f();
      if (mag_declination > M_PI) { // Angle normalization (-180 deg to 180 deg)
        mag_declination -= (2.0 * M_PI);
      } else if (mag_declination < -M_PI) { mag_declination += (2.0 * M_PI); }
      mag_declination -= mag_heading_rad;
      declination_calculated = true;
      if (fabs(mag_declination) > RadOfDeg(10.)) { mag_declination = 0; declination_calculated = false; }
    }
  }
  mag_heading_rad = mag_heading_rad + mag_declination;
  if (mag_heading_rad > M_PI) { // Angle normalization (-180 deg to 180 deg)
    mag_heading_rad -= (2.0 * M_PI);
  } else if (mag_heading_rad < -M_PI) { mag_heading_rad += (2.0 * M_PI); }
  // Magnetic COMPASS Heading N = 0, E = 90, S = 180, W = 270
  mag_course_deg = DegOfRad(mag_heading_rad);
  if (mag_course_deg < 0) { mag_course_deg += 360; }

  return;
}
#endif


static void send_mag_heading(struct transport_tx *trans, struct link_device *dev)
{
  uint8_t abi_id = ABI_BROADCAST;
#if DOWNLINK
  pprz_msg_send_IMU_MAG(trans, dev, AC_ID, &abi_id, &mag_course_deg, &gps_course_deg, &home_dir_deg);
#endif

  return;
}

//*******************************************************************
//   function name:   vSubtractVectors
//   description:     subtracts two vectors a = b - c
//   parameters:
//*******************************************************************
static void vSubtractVectors(VECTOR *svA, VECTOR svB, VECTOR svC)
{
  svA->fx = svB.fx - svC.fx;
  svA->fy = svB.fy - svC.fy;
  svA->fz = svB.fz - svC.fz;
}

//*******************************************************************
// function name:   vMultiplyMatrixByVector
// description:     multiplies matrix by vector svA = smB * svC
// parameters:
//*******************************************************************
static void vMultiplyMatrixByVector(VECTOR *svA, MATRIX smB, VECTOR svC)
{
  svA->fx = smB.fx1 * svC.fx  +  smB.fx2 * svC.fy  +  smB.fx3 * svC.fz;
  svA->fy = smB.fy1 * svC.fx  +  smB.fy2 * svC.fy  +  smB.fy3 * svC.fz;
  svA->fz = smB.fz1 * svC.fx  +  smB.fz2 * svC.fy  +  smB.fz3 * svC.fz;
}


static float home_direction(void)
{

  static VECTOR svPlanePosition, Home_Position, Home_PositionForPlane, Home_PositionForPlane2;
  static MATRIX smRotation;
  float home_dir = 0;


  /*
  By swapping coordinates (fx=fPlaneNorth, fy=fPlaneEast) we make the the circle angle go from 0 (0 is to the top of the circle)
  to 360 degrees or from 0 radians to 2 PI radians in a clockwise rotation. This way the GPS reported angle can be directly
  applied to the rotation matrices (in radians).
  In standard mathematical notation 0 is to the right (East) of the circle, -90 is to the bottom, +-180 is to the left
  and +90 is to the top (counterclockwise rotation).
  When reading back the actual rotated coordinates fx has the y coordinate and fy has the x when
  represented on a circle in standard mathematical notation.
  */
  if (gps.fix == GPS_FIX_3D && stateGetHorizontalSpeedNorm_f() > 5.0) { //Only when flying
    svPlanePosition.fx = stateGetPositionEnu_f()->y;
    svPlanePosition.fy = stateGetPositionEnu_f()->x;
    svPlanePosition.fz = stateGetPositionUtm_f()->alt;
#if defined(FIXEDWING_FIRMWARE)
    Home_Position.fx = WaypointY(WP_HOME);
    Home_Position.fy = WaypointX(WP_HOME);
    Home_Position.fz = GetAltRef();
#else
    Home_Position.fx = waypoint_get_x(WP_HOME);
    Home_Position.fy = waypoint_get_y(WP_HOME);
    Home_Position.fz = 0;
#endif

    /* distance between plane and object */
    vSubtractVectors(&Home_PositionForPlane, Home_Position, svPlanePosition);

    /* yaw */
    smRotation.fx1 = cosf(stateGetHorizontalSpeedDir_f());
    smRotation.fx2 = sinf(stateGetHorizontalSpeedDir_f());
    smRotation.fx3 = 0.;
    smRotation.fy1 = -smRotation.fx2;
    smRotation.fy2 = smRotation.fx1;
    smRotation.fy3 = 0.;
    smRotation.fz1 = 0.;
    smRotation.fz2 = 0.;
    smRotation.fz3 = 1.;

    vMultiplyMatrixByVector(&Home_PositionForPlane2, smRotation, Home_PositionForPlane);

    /* DEFAULT ORIENTATION IS 0 = FRONT, 90 = RIGHT, 180 = BACK, -90 = LEFT
     *
     * WHEN home_dir =  (float)(atan2(Home_PositionForPlane2.fy, (Home_PositionForPlane2.fx)));
     *
     *             plane front
     *                  0˚
     *                  ^
     *                  I
     *             -45˚ I  45˚
     *                \ I /
     *                 \I/
     *       -90˚-------I------- 90˚
     *                 /I\
     *                / I \
     *            -135˚ I  135˚
     *                  I
     *                 180
     *             plane back
     *
     *
     * When the home_dir variable goes to 0 the aircraft is headed straight back home
    */

    /* fixed to the plane*/
    home_dir = atan2(Home_PositionForPlane2.fy, Home_PositionForPlane2.fx);
    if (home_dir > M_PI) { // Angle normalization (-180 deg to 180 deg but still in radians)
      home_dir -= (2.0 * M_PI);
    } else if (home_dir < -M_PI) { home_dir += (2.0 * M_PI); }
    home_dir_deg = DegOfRad(home_dir); // Now convert radians to degrees.

  } // END OF if (gps.fix == GPS_FIX_3D) statement.

  return (home_dir_deg);
}

static char ascii_to_osd_c(char c)
{

#if defined USE_MATEK_TYPE_OSD_CHIP && USE_MATEK_TYPE_OSD_CHIP == 1
  PRINT_CONFIG_MSG("OSD USES THE CUSTOM MATEK TYPE OSD CHIP")

  return (c);

#else
  PRINT_CONFIG_MSG("OSD USES THE STANDARF MAX7456 OSD CHIP")

  if (c >= '0' && c <= '9') {
    if (c == '0') { c -= 38; } else {  c -= 48; }
  } else if (c >= 'A' && c <= 'Z') {
    c -= 54;
  } else if (c >= 'a' && c <= 'z') {
    c -= 60;

  } else {
    switch (c) {
      case ('('): c = 0x3f; break;
      case (')'): c = 0x40; break;
      case ('.'): c = 0x41; break;
      case ('?'): c = 0x42; break;
      case (';'): c = 0x43; break;
      case (':'): c = 0x44; break;
      case (','): c = 0x45; break;
      //case('''): c = 0x46; break;
      case ('/'): c = 0x47; break;
      case ('"'): c = 0x48; break;
      case ('-'): c = 0x49; break;
      case ('<'): c = 0x4A; break;
      case ('>'): c = 0x4B; break;
      case ('@'): c = 0x4C; break;
      case (' '): c = 0x00; break;
      case ('\0'): c = 0xFF; break;
      default  : break;
    }
  }

  return (c);

#endif
}

static void calc_flight_time_left(void)
{
  float current_amps = 0;
  float horizontal_speed = 0;
  float bat_capacity_left = 0;
  static float bat_capacity_used = 0;


  current_amps = electrical.current;
  bat_capacity_used += (current_amps * 1000.) / (3600. * (float)MAX7456_PERIODIC_FREQ);
  bat_capacity_left = (float)BAT_CAPACITY - bat_capacity_used;
  if (bat_capacity_left < 0) { bat_capacity_left = 0; }
  horizontal_speed = stateGetHorizontalSpeedNorm_f();

#if defined(FIXEDWING_FIRMWARE)
  if (stateGetHorizontalSpeedNorm_f() < 5.0 || autopilot.launch == false) {
    current_amps = LOITER_BAT_CURRENT;
    horizontal_speed = MINIMUM_AIRSPEED;
  }
#else // #if !FW
  current_amps = 1.0; // FIXME, Find how to tell if the rotorcraft is on the ground or it is flying.
  horizontal_speed = 10.0;
#endif

#if defined(OSD_USE_FLOAT_LOW_PASS_FILTERING)

  static double current_amps_sum = 0;
  static double horizontal_speed_sum = 0;
  static float current_amps_filtered = 0;
  static float horizontal_speed_filtered = 0;

  current_amps_sum = (current_amps_sum - current_amps_filtered) + current_amps;
  current_amps_filtered = current_amps_sum / (1 << AMPS_LOW_PASS_FILTER_STRENGTH);
  current_amps = current_amps_filtered;

  horizontal_speed_sum = (horizontal_speed_sum - horizontal_speed_filtered) + horizontal_speed;
  horizontal_speed_filtered = horizontal_speed_sum / (1 << SPEED_LOW_PASS_FILTER_STRENGTH);
  horizontal_speed = horizontal_speed_filtered;

#else

  uint32_t current_amps_int = 0;
  uint32_t horizontal_speed_int = 0;
  static uint64_t current_amps_sum = 0;
  static uint64_t horizontal_speed_sum = 0;
  static uint32_t current_amps_filtered = 0;
  static uint32_t horizontal_speed_filtered = 0;

  // LOW PASS FILTERS for making the OSD 'max_flyable_distance_left' var change more gently.
  current_amps_int = (uint32_t)(current_amps * 1000.0);
  horizontal_speed_int = (uint32_t)(horizontal_speed * 1000.0);

  current_amps_sum = (current_amps_sum - current_amps_filtered) + current_amps_int;
  current_amps_filtered = (current_amps_sum + (1 << (AMPS_LOW_PASS_FILTER_STRENGTH - 1))) >>
                          (AMPS_LOW_PASS_FILTER_STRENGTH);

  horizontal_speed_sum = (horizontal_speed_sum - horizontal_speed_filtered) + horizontal_speed_int;
  horizontal_speed_filtered = (horizontal_speed_sum + (1 << (SPEED_LOW_PASS_FILTER_STRENGTH - 1))) >>
                              (SPEED_LOW_PASS_FILTER_STRENGTH);

  current_amps = (float)(current_amps_filtered) / 1000.;
  horizontal_speed = (float)(horizontal_speed_filtered) / 1000.;

#endif

  max_flyable_distance_left = ((bat_capacity_left / (current_amps * 1000)) * 3600) * horizontal_speed;

  return;
}


static void osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)
{

  int8_t x = 0, idx = 0, post_offset = 0, aft_offset = 0, string_len = 0;
  char osd_buf[OSD_STRING_SIZE];

  if (row > 15) { column = 15; }
  if (column > 29) { column = 29; }

// translate the string and put it to the "osd_string" '\0' = 0xff
  x = 0;
  while (*(string + x) != '\0') { osd_string[x] = ascii_to_osd_c(*(string + x)); x++; }
  osd_string[x] = 0xff;
  string_len = x;
  idx = x;

  if (attributes & C_JUST) {
    if (char_nb % 2 == 0) { char_nb++; }
    post_offset = (char_nb - string_len) / 2;
    aft_offset = char_nb - string_len;
    if (((int8_t)column - (char_nb / 2)) >= 0) { column -= (char_nb / 2); }
    for (x = 0; x < 30; x++) { osd_buf[x] = 0; } // FILL WITH SPACES
    // COPY THE ORIGINAL STRING TO ITS NEW POSITION
    for (x = 0; x < string_len; x++) { osd_buf[post_offset + x] = osd_string[x]; }
    osd_buf[string_len + aft_offset] = 0xFF; // TERMINATE THE MODIFIED STRING
    // COPY THE MODIFIED STRING TO MAIN OSD STRING
    x = 0;
    do { osd_string[x] = osd_buf[x]; } while (osd_buf[x++] != 0xFF);
  } else if (attributes & R_JUST) {
    //if(x){ x -= 1; }
    //if (char_nb < string_len){ char_nb = string_len; }
    if (((int8_t)column - char_nb) >= 0) { column -= char_nb; }
    if (((int8_t)char_nb - string_len) >= 0) { post_offset = char_nb - string_len; } else {post_offset = 0; }
    //ADD LEADING SPACES
    //First shift right the string and then add spaces at the beggining
    while (idx >= 0) { osd_string[idx + post_offset] = osd_string[idx]; idx--; }
    idx = 0;
    while (idx < post_offset) { osd_string[idx] = 0; idx++; }
    //osd_string[idx] = 0xff;

  } else {
    //Adjust for the reserved character number.
    for (x = 0; x < (int8_t)(sizeof(osd_string)); x++) { if (osd_string[x] == 0xFF) { break; } }
    for (; x < char_nb; x++) {  osd_string[x] = 0; }
    osd_string[x] = 0xff;
  }

  osd_char_address = ((uint16_t)row * 30) + column;
  osd_attr = (attributes & (BLINK | INVERT));
//TRIGGER THE SPI TRANSFERS. The rest of the spi transfers occur in the "max7456_event" function.
  if (max7456_osd_status == OSD_IDLE) {
    max7456_trans.output_length = 2;
    max7456_trans.output_buf[0] = OSD_DMAH_REG;
    max7456_trans.output_buf[1] = (uint8_t)((osd_char_address >> 8) & 0x0001);
    max7456_osd_status = OSD_S_STEP1;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);

  }

  return;
}


static bool _osd_sprintf(char *buffer, char *string, float value)
{
  uint8_t param_start = 0;
  uint8_t param_end = 0;
  uint8_t frac_nb = 0;
  uint8_t digit = 0;
  uint8_t x = 0, y = 0, z = 0;

  uint16_t i_dec = 0;
  uint16_t i_frac = 0;

  char to_asc[10] = {48, 48, 48, 48, 48, 48, 48, 48, 48, 48};
  char string_buf[OSD_STRING_SIZE];

// Clear the osd string.
  for (x = 0; x < sizeof(osd_string); x++) { osd_string[x] = 0; }
  for (x = 0; x < sizeof(string_buf); x++) { string_buf[x] = 0; }

//copy the string passed as parameter to a buffer
  for (x = 0; x < sizeof(string_buf); x++) { string_buf[x] = *(string + x); if (string_buf[x] == '\0') { break; } }
  do {
    x = 0;
    param_start = 0;
    param_end = 0;
    //Now check for any special character
    while (string_buf[x] != '\0') {
      // EXAMPLE: in "%160c"x is '%' x+4 = 'c' and x+1='1', x+2='6' and x+3='0'
      if (string_buf[x] == '%') { if (string_buf[x + 4] == 'c') { (param_start = x + 1); param_end = x + 3; break; } }
      x++;
    }
    if (param_end - param_start) {
      //load the special character value where the % character was
      string_buf[x] = ((string_buf[param_start] - 48) * 100) + ((string_buf[param_start + 1] - 48) * 10) +
                      (string_buf[param_start + 2] - 48);
      x++; // increment x to the next character which should be the first special character's digit
      //Move the rest of the buffer forward so only the special character remains,
      // for example in %170c '%' now has the special character's code and x now points to '1'
      // which will be overwritten with the rest of the string after the 'c'
      for (y = (x + 4); y <= sizeof(string_buf); y++) { string_buf[x++] = string_buf[y]; }
    }
  } while ((param_end - param_start > 0));

// RESET THE USED VARIABLES JUST TO BE SAFE.
  x = 0;
  y = 0;
  param_start = 0;
  param_end = 0;
// Search for the prameter start and stop positions.
  while (string_buf[x] != '\0') {
    if (string_buf[x] == '%') {
      param_start = x;

    } else if (string_buf[x] == 'f') { param_end = x; break; }
    x++;
  }
  if (param_end - param_start) {
    // find and bound the precision specified.
    frac_nb =  string_buf[param_end - 1] - 48; // Convert to number, ASCII 48 = '0'
    if (frac_nb > 3) { frac_nb = 3; }     // Bound value.

    y = (sizeof(to_asc) - 1); // Point y to the end of the array.
    i_dec = abs((int16_t)value);
    // Fist we will deal with the fractional part if specified.
    if (frac_nb > 0 && frac_nb <= 3) {
      i_frac = abs((int16_t)((value - (int16_t)value) * 1000)); // Max precision is 3 digits.
      x = 100;
      z = frac_nb;
      do {                           // Example if frac_nb=2 then 952 will show as .95
        z--;
        digit = (i_frac / x);
        to_asc[y + z] = digit + 48; // Convert to ASCII
        i_frac -= digit * x;      // Calculate the remainder.
        x /= 10;                  // 952-(9*100) = 52, 52-(10*5)=2 etc.

      } while (z > 0);

      y -= frac_nb;     // set y to point where the dot must be placed.
      to_asc[y] = '.';
      y--;             // Set y to point where the rest of the numbers must be written.

    }  // if (frac_nb > 0 && frac_nb <= 3){

    // Now it is time for the integer part. "y" already points to the position just before the dot.
    do {
      to_asc[y] = (i_dec % 10) + 48;            //Write at least one digit even if value is zero.
      i_dec /= 10;
      if (i_dec <= 0) {                         // This way the leading zero is ommited.
        if (value < 0) { y--; to_asc[y] = '-'; } // Place the minus sign if needed.
        break;

      } else { y--; }

    } while (1);

    // Fill the buffer with the characters in the beggining of the string if any.
    for (x = 0; x < param_start; x++) { *(buffer + x) = string_buf[x]; }

    // x is now pointing to the next character in osd_string.
    // y is already pointing to the first digit or negative sign in "to_asc" array.
    while (y < sizeof(to_asc)) { *(buffer + x) = to_asc[y]; x++; y++; }
    // x is now pointing to the next character in osd_string.
    // "param_end" is pointing to the last format character in the string.
    do {
      param_end++;
      *(buffer + x++) = string_buf[param_end];

    } while (string_buf[param_end] != '\0'); //Write the rest of the string including the terminating char.

    // End of if (param_end - param_start)
  } else {
    for (x = 0; x < sizeof(string_buf); x++) {
      *(buffer + x) = string_buf[x]; //Write the rest of the string including the terminating char.
      if (*(buffer + x) == '\0') { break; }
    }
  }

  return (0);
}

void draw_osd(void)
{
  float temp = 0;
  float altitude = 0;
  float distance_to_home = 0;
  static float home_direction_degrees = 0;
#if defined(BARO_ALTITUDE_VAR)
  static float baro_alt_correction = 0;
#endif

  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct EnuCoor_f *pos = stateGetPositionEnu_f();
#if defined(FIXEDWING_FIRMWARE)
  float ph_x = waypoints[WP_HOME].x - pos->x;
  float ph_y = waypoints[WP_HOME].y - pos->y;
  float stall_speed = STALL_AIRSPEED;
#else // FOR ROTORCRAFTS
  float ph_x = waypoint_get_x(WP_HOME) - pos->x;
  float ph_y = waypoint_get_y(WP_HOME) - pos->y;
#endif //

  distance_to_home = (float)(sqrt(ph_x * ph_x + ph_y * ph_y));
  calc_flight_time_left();
#if defined(FIXEDWING_FIRMWARE)
  mag_compass();
#endif

  //THE SWITCH STATEMENT ENSURES THAT ONLY ONE SPI TRANSACTION WILL OCUUR IN EVERY PERIODIC FUNCTION CALL
  switch (step) {

    case (0):
      if (gps.fix == GPS_FIX_3D && stateGetHorizontalSpeedNorm_f() > 10.0) { //Only when flying
        gps_course_deg = (float)gps.course;
        gps_course_deg = DegOfRad(gps_course_deg / 1e7);
      } else {
#if defined(FIXEDWING_FIRMWARE)
        gps_course_deg = mag_course_deg;
#else
        gps_course_deg = stateGetNedToBodyEulers_f()->psi;
#endif
      }
      osd_sprintf(osd_string, "%.0f", gps_course_deg);
      osd_put_s(osd_string, C_JUST, 3, 1, 15);
      step = 10;
      break;

    case (10):
      //Only when flying because i need this indication to remain stable if the GPS is lost.
      // This way i can still have the synchronized magnetic compass heading and the last bearing home.
      if (gps.fix == GPS_FIX_3D && gps.pdop < 1000 && stateGetHorizontalSpeedNorm_f() > 5.0) { //Only when flying
        home_direction_degrees = gps_course_deg + home_direction(); //home_direction returns degrees -180 to +180
        if (home_direction_degrees < 0) { home_direction_degrees += 360; } // translate the -180, +180 to 0-360.
        if (home_direction_degrees >= 360) { home_direction_degrees -= 360; }
      }
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      // All special character codes must be in 3 digit format!
      osd_sprintf(osd_string, "%191c%.0f", home_direction_degrees); // 0 when heading straight back home.
      osd_put_s(osd_string, C_JUST, 5, 2, 15); // "false = L_JUST
#else
      osd_sprintf(osd_string, "H%.0f", home_direction_degrees);
      osd_put_s(osd_string, C_JUST, 5, 2, 15); // "false = L_JUST

#endif
      step = 20;
      break;

    case (20):
      temp = ((float)electrical.vsupply);
      osd_sprintf(osd_string, "%.1fV", temp);
      if (temp > LOW_BAT_LEVEL) {
        osd_put_s(osd_string, L_JUST, 5, 1, 1);

      } else { osd_put_s(osd_string, (L_JUST | BLINK | INVERT), 5, 1, 1); }
      step = 30;
      break;

    case (30):
      if (gps.fix == GPS_FIX_3D) {
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
        //Since we only send one special character the float variable is replaced by a zero.
        osd_sprintf(osd_string, "%030c%031c", 0);
        osd_put_s(osd_string, false, 2, 2, 1);
#else
        osd_put_s("**", false, 2, 2, 1);
#endif
      } else {
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
        //Since we only send one special character the float variable is replaced by a zero.
        osd_sprintf(osd_string, "%030c%031c", 0); // ALL special osd chars must have 3 digits.
        osd_put_s(osd_string, (L_JUST | BLINK), 2, 2, 1);
#else
        osd_put_s("**", (L_JUST | BLINK), 2, 2, 1);
#endif
      }
      step = 40;
      break;

    case (40):
#if defined(FIXEDWING_FIRMWARE)
      if (autopilot.mode == AP_MODE_AUTO2) {
        osd_put_s("A2", L_JUST, 2, 2, 3);
      } else if (autopilot.mode == AP_MODE_AUTO1) {
        osd_put_s("A1", L_JUST, 2, 2, 3);
      } else {
        osd_put_s("MAN", L_JUST, 3, 2, 3);
      }
#endif
      step = 50;
      break;

    case (50):
#if defined(FIXEDWING_FIRMWARE)
      osd_sprintf(osd_string, "THR%.0f", (((float)command_get(COMMAND_THROTTLE) / (float)MAX_PPRZ) * 100.));
#else
      osd_sprintf(osd_string, "THR%.0fTHR", (((float)stabilization_cmd[COMMAND_THRUST] / (float)MAX_PPRZ) * 100.));
#endif
      osd_put_s(osd_string, L_JUST, 6, 3, 1);
      step = 60;
      break;

    case (60):
#if defined(FIXEDWING_FIRMWARE)
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      if ((fabs(stateGetHorizontalSpeedNorm_f() * cos(att->phi))) < stall_speed) {
        osd_sprintf(osd_string, "STALL!", 0);
        osd_put_s(osd_string, (R_JUST | BLINK), 6, 1, 30);
      } else {
        osd_sprintf(osd_string, "%.0f%161c", (stateGetHorizontalSpeedNorm_f() * 3.6));
        osd_put_s(osd_string, R_JUST, 6, 1, 30);
      }
#else
      if ((fabs(stateGetHorizontalSpeedNorm_f() * cos(att->phi))) < stall_speed) {
        osd_sprintf(osd_string, "STALL!", 0);
        osd_put_s(osd_string, (R_JUST | BLINK), 6, 1, 30);
      } else {
        osd_sprintf(osd_string, "%.0fKM", (stateGetHorizontalSpeedNorm_f() * 3.6));
        osd_put_s(osd_string, R_JUST, 6, 1, 30);
      }
#endif

#else // #if !FW

#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      osd_sprintf(osd_string, "%.0f%161c", (stateGetHorizontalSpeedNorm_f() * 3.6));
#else
      osd_sprintf(osd_string, "%.0fKM", (stateGetHorizontalSpeedNorm_f() * 3.6));
#endif
      osd_put_s(osd_string, R_JUST, 6, 1, 30);

#endif
      step = 70;
      break;

    case (70):
#if defined(BARO_ALTITUDE_VAR)
      if (gps.fix == GPS_FIX_3D && gps.pdop < 1000) {
        RunOnceEvery((MAX7456_PERIODIC_FREQ * 300), { baro_alt_correction = GetPosAlt() - BARO_ALTITUDE_VAR;});
        altitude = GetPosAlt();
      } else {
        altitude = BARO_ALTITUDE_VAR + baro_alt_correction;
      }
#else
      altitude = GetPosAlt();
#endif
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      osd_sprintf(osd_string, "%.0f%177c", altitude);
#else
      osd_sprintf(osd_string, "%.0fM", altitude);
#endif
      osd_put_s(osd_string, R_JUST, 6, 2, 30); // "false = L_JUST
      step = 80;
      break;

    case (80):
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      osd_sprintf(osd_string, "%.1f%159c", stateGetSpeedEnu_f()->z);
#else
      osd_sprintf(osd_string, "%.1fVZ", stateGetSpeedEnu_f()->z);
#endif
      if (stateGetSpeedEnu_f()->z > 3.0) {
        osd_put_s(osd_string, (R_JUST | BLINK), 6, 3, 30);
      } else {
        osd_put_s(osd_string, R_JUST, 6, 3, 30);
      }
      step = 90;
      break;

    case (90):
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      // ANY SPECIAL CHARACTER CODE MUST BE A 3 DIGIT NUMBER WITH THE LEADING ZEROS!!!!
      // THE SPECIAL CHARACTER CAN BE PLACED BEFORE OR AFTER THE FLOAT OR ANY OTHER CHARACTER
      osd_sprintf(osd_string, "%160c%.1fK%012c", (distance_to_home / 1000));
      osd_put_s(osd_string, L_JUST, 7, 15, 1);
#else
      osd_sprintf(osd_string, "%.1fKM", (distance_to_home / 1000));
      osd_put_s(osd_string, L_JUST, 7, 15, 1);
#endif
      step = 100;
      break;

    case (100):
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      osd_sprintf(osd_string, "%147c%.1fK%012c", (max_flyable_distance_left / 1000));
#else
      osd_sprintf(osd_string, "%.1fKM", (max_flyable_distance_left / 1000));
#endif
      if ((max_flyable_distance_left + 1000.0) < distance_to_home) {
        osd_put_s(osd_string, (R_JUST | BLINK), 7, 15, 30);
      } else {
        osd_put_s(osd_string, (R_JUST), 7, 15, 30);
      }
      step = 110;
      break;

    // A Text PFD as graphics are not the strong point of the MAX7456
    // In order to level the aircraft while fpving
    // just move the stick to the opposite direction from the angles shown on the osd
    // and that's why positive pitch (UP) is shown below the OSD center
    case (110):
      if (DegOfRad(att->theta) > 3) {
        osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
        osd_put_s(osd_string, C_JUST, 5, 6, 15);

      } else { osd_put_s("    ", C_JUST, 5, 6, 15); }
      step = 112;
      break;

    case (112):
      if (DegOfRad(att->theta) < -3) {
        osd_sprintf(osd_string, "%.0f", DegOfRad(att->theta));
        osd_put_s(osd_string, C_JUST, 5, 10, 15);

      } else { osd_put_s("   ", C_JUST, 5, 10, 15); }
      step = 114;
      break;

    case (114):
      if (DegOfRad(att->phi) > 3) {
        osd_sprintf(osd_string, "%.0f>", DegOfRad(att->phi));
        osd_put_s(osd_string, false, 5, 8, 18);

      } else { osd_put_s("     ", false, 5, 8, 18); }
      step = 116;
      break;

    case (116):
      if (DegOfRad(att->phi) < -3) {
        osd_sprintf(osd_string, "<%.0f", DegOfRad(fabs(att->phi)));
        osd_put_s(osd_string, R_JUST, 5, 8, 13);

      } else { osd_put_s("     ", R_JUST, 5, 8, 13); }
      step = 120;
      break;

    case (120):
#if defined(USE_MATEK_TYPE_OSD_CHIP) && USE_MATEK_TYPE_OSD_CHIP == 1
      osd_sprintf(osd_string, "%126c", 0);
      osd_put_s(osd_string, false, 1, 8, 15); // false = L_JUST
#else
      osd_put_s("+", false, 1, 8, 15); // false = L_JUST
#endif
      step = 0;
      break;

    default: step = 0; break;
  }  // End of switch statement.

  return;
}

void max7456_init(void)
{

  max7456_trans.slave_idx     = MAX7456_SLAVE_IDX;
  max7456_trans.select        = SPISelectUnselect;
  max7456_trans.cpol          = SPICpolIdleLow;
  max7456_trans.cpha          = SPICphaEdge1;
  max7456_trans.dss           = SPIDss8bit;
  max7456_trans.bitorder      = SPIMSBFirst;
  max7456_trans.cdiv          = SPIDiv64;
  max7456_trans.output_length = sizeof(osd_spi_tx_buffer);
  max7456_trans.output_buf    = (uint8_t *) osd_spi_tx_buffer;
  max7456_trans.input_length  = 0;
  max7456_trans.input_buf     = (uint8_t *)osd_spi_rx_buffer;
  max7456_trans.before_cb     = NULL;
  max7456_trans.after_cb      = NULL;

  osd_enable = 1;
  osd_enable_val = OSD_IMAGE_ENABLE;
  max7456_osd_status = OSD_UNINIT;

#if DOWNLINK
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_IMU_MAG, send_mag_heading);
#endif
  home_direction();

  return;
}

void max7456_periodic(void)
{

  //This code is executed always and checks if the "osd_enable" var has been changed by telemetry.
  //If yes then it commands a reset but this time turns on or off the osd overlay, not the video.
  if (max7456_osd_status == OSD_IDLE) {
    if (osd_enable > 1) {
      osd_enable = 1;
    }
    if ((osd_enable << 3) !=  osd_enable_val) {
      osd_enable_val = (osd_enable << 3);
      max7456_osd_status = OSD_UNINIT;
    }
  }

  //INITIALIZATION OF THE OSD
  if (max7456_osd_status == OSD_UNINIT) {
    step = 0;
    max7456_trans.status = SPITransDone;
    max7456_trans.output_buf[0] = OSD_VM0_REG;
    //This operation needs at least 100us but when the periodic function will be invoked again
    //sufficient time will have elapsed even with at a periodic frequency of 1000 Hz
    max7456_trans.output_buf[1] = OSD_RESET;
    //We give an extra delay step by going to the event function and back here for the Reset to complete.
    max7456_osd_status = OSD_INIT1;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  } else if (max7456_osd_status == OSD_INIT2) {
    max7456_trans.output_length = 1;
    max7456_trans.input_length = 1;
    max7456_trans.output_buf[0] = OSD_OSDBL_REG_R;
    max7456_osd_status = OSD_INIT3;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  } else if (max7456_osd_status == OSD_IDLE && osd_enable > 0) {
    draw_osd();
  }  // End of if (max7456_osd_status == OSD_UNINIT)



  return;
}

void max7456_event(void)
{

  static uint8_t x = 0;

  if (max7456_trans.status == SPITransSuccess) {
    max7456_trans.status = SPITransDone;

    switch (max7456_osd_status) {
      case (OSD_INIT1):
        max7456_osd_status = OSD_INIT2;
        break;
      case (OSD_INIT3):
        max7456_trans.output_length = 2;
        max7456_trans.input_length = 0;
        max7456_trans.output_buf[0] = OSD_OSDBL_REG;
        //Max7456 requires that you read first and then change only bit4 of the OSDBL register.
        //Reading was started in the periodic function and now we rerwrite the OSDBL register.
        max7456_trans.output_buf[1] = max7456_trans.input_buf[1] & (~(1 << 4));
        max7456_osd_status = OSD_INIT4;
        spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        break;
      case (OSD_INIT4):
        max7456_trans.output_buf[0] = OSD_VM0_REG;
#if USE_PAL_FOR_OSD_VIDEO
#pragma message "Camera and OSD must be both PAL or NTSC otherwise only the camera picture will be visible."
        max7456_trans.output_buf[1] = OSD_VIDEO_MODE_PAL | osd_enable_val;
#else
        max7456_trans.output_buf[1] = osd_enable_val;
#endif
        max7456_osd_status = OSD_FINISHED;
        spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        break;
      case (OSD_S_STEP1):
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
        if (osd_string[x] != 0XFF) {
          max7456_trans.output_buf[0] = osd_string[x++];
          spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        } else {
          max7456_trans.output_buf[0] = 0xFF; //Exit the auto increment mode
          max7456_osd_status = OSD_FINISHED;
          spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        }
        break;
      case (OSD_FINISHED):
        max7456_trans.output_length = 1;
        max7456_trans.input_length = 2;
        max7456_trans.output_buf[0] = OSD_STAT_REG;
        max7456_osd_status = OSD_READ_STATUS;
        spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        break;
      case (OSD_READ_STATUS):
        osd_stat_reg = max7456_trans.input_buf[1];
        if (osd_stat_reg & (OSD_NVRAM_BUSY_FLAG | OSD_RESET_BUSY_FLAG)) {
          max7456_trans.output_length = 1;
          max7456_trans.input_length = 2;
          max7456_trans.output_buf[0] = OSD_STAT_REG;
          max7456_osd_status = OSD_READ_STATUS;
          spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        } else {
          osd_attr = 0;
          max7456_trans.status = SPITransDone;
          max7456_osd_status = OSD_IDLE;
        }
        break;

      default: break;
    }
  }
  return;
}
