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

/**
 * @file modules/display/max7456.c
 * Maxim MAX7456 single-channel monochrome on-screen display driver.
 *
 */

#include "std.h"
//#include "stdio.h"

#include "mcu_periph/sys_time.h"
#include "mcu_periph/gpio.h"
#include "mcu_periph/spi.h"

#include "generated/flight_plan.h"
#include "generated/airframe.h"
#include "autopilot.h"
#include "subsystems/electrical.h"
#include "state.h"

// for GetPosAlt, include correct header until we have unified API
#ifdef AP
#include "subsystems/navigation/nav.h"
#else
#include "firmwares/rotorcraft/navigation.h"
#endif

// Peripherials
#include "modules/display/max7456.h"
#include "modules/display/max7456_regs.h"

#define OSD_STRING_SIZE     31
#define osd_sprintf         _osd_sprintf

static char ascii_to_osd_c(char c);
static void osd_put_s(char *string,  uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column);
static bool_t _osd_sprintf(char *buffer, char *string, float value);

struct spi_transaction max7456_trans;

uint8_t osd_spi_tx_buffer[2];
uint8_t osd_spi_rx_buffer[2];
char osd_string[OSD_STRING_SIZE];
char osd_str_buf[OSD_STRING_SIZE];
char osd_char = ' ';
uint8_t step = 0;
uint16_t osd_char_address = 0;
uint8_t  osd_attr = FALSE;

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
};

uint8_t max7456_osd_status = OSD_UNINIT;
uint8_t osd_enable = TRUE;
uint8_t osd_enable_val = OSD_IMAGE_ENABLE;
uint8_t osd_stat_reg = 0;
bool_t  osd_stat_reg_valid = FALSE;

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

  return;
}

void max7456_periodic(void)
{

  float temp = 0;
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
    max7456_osd_status = OSD_INIT1;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  } else if (max7456_osd_status == OSD_INIT2) {
    max7456_trans.output_length = 1;
    max7456_trans.input_length = 1;
    max7456_trans.output_buf[0] = OSD_OSDBL_REG_R;
    max7456_osd_status = OSD_INIT3;
    spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
  } else if (max7456_osd_status == OSD_IDLE && osd_enable > 0) { // DRAW THE OSD SCREEN
    //draw_osd();
    switch (step) {
      case (0):
        osd_put_s("HDG", FALSE, 3, 0, 13);
        step = 10;
        break;
      case (10):
        temp = ((float)electrical.vsupply) / 10;
        osd_sprintf(osd_string, "%.1fV", temp);
        if (temp > LOW_BAT_LEVEL) {
          osd_put_s(osd_string, FALSE, 8, 0, 2);
        } else {
          osd_put_s(osd_string, BLINK | INVERT, 8, 0, 2);
        }
        step = 20;
        break;
      case (20):
#if MAG_HEADING_AVAILABLE && !defined(SITL)
        temp = DegOfRad(MAG_Heading);
        if (temp < 0) {
          temp += 360;
        }
#else
        temp = DegOfRad(state.h_speed_dir_f);
        if (temp < 0) {
          temp += 360;
        }
#endif
        osd_sprintf(osd_string, "%.0f", temp);
        osd_put_s(osd_string, FALSE, 8, 1, 13);
        step = 30;
        break;
      case (30):
        osd_sprintf(osd_string, "%.0fKm", (state.h_speed_norm_f * 3.6));
        osd_put_s(osd_string, FALSE, 8, 0, 24);
        step = 40;
        break;
      case (40):
        osd_sprintf(osd_string, "%.0fm", GetPosAlt());
        osd_put_s(osd_string, FALSE, 10, 13, 2);
        step = 50;
        break;
      case (50):
        osd_sprintf(osd_string, "%.1fVZ", stateGetSpeedEnu_f()->z);
        osd_put_s(osd_string, FALSE, 7, 13, 24);
        step = 10;
        break;
      default:  break;
    }
  }
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
        max7456_trans.output_buf[1] = max7456_trans.input_buf[0] & (~(1 << 4));
        max7456_osd_status = OSD_INIT4;
        spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        break;
      case (OSD_INIT4):
        max7456_trans.output_buf[0] = OSD_VM0_REG;
#if USE_PAL_FOR_OSD_VIDEO
        max7456_trans.output_buf[1] = OSD_VIDEO_MODE_PAL | osd_enable_val;
#else
        max7456_trans.output_buf[1] = osd_enable_val;
#endif
        max7456_osd_status = OSD_FINISHED;
        spi_submit(&(MAX7456_SPI_DEV), &max7456_trans);
        break;
      case (OSD_READ_STATUS):
        osd_stat_reg = max7456_trans.input_buf[0];
        osd_stat_reg_valid = TRUE;
        max7456_osd_status = OSD_FINISHED;
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
        osd_attr = 0;
        max7456_trans.status = SPITransDone;
        max7456_osd_status = OSD_IDLE;
        break;
      default: break;
    }
  }
  return;
}

static char ascii_to_osd_c(char c)
{

  if (c >= '0' && c <= '9') {
    if (c == '0') {
      c -= 38;
    } else {
      c -= 48;
    }
  } else {
    if (c >= 'A' && c <= 'Z') {
      c -= 54;
    } else {
      if (c >= 'a' && c <= 'z') {
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
    }
  }
  return (c);
}

static void osd_put_s(char *string, uint8_t attributes, uint8_t char_nb, uint8_t row, uint8_t column)
{

  uint8_t x = 0;

  if (row > 15) {
    column = 15;
  }
  if (column > 29) {
    column = 29;
  }
  osd_char_address = ((uint16_t)row * 30) + column;

  // translate the string and put it to the "osd_string" '\0' = 0xff
  x = 0;
  while (*(string + x) != '\0') {
    osd_string[x] = ascii_to_osd_c(*(string + x));
    x++;
  }
  osd_string[x] = ascii_to_osd_c(*(string + x));

  for (x = 0; x < sizeof(osd_string); x++) {
    if (osd_string[x] == 0xff) {
      break;
    }
  }

  //Adjust for the reserved character number.
  for (; x < char_nb; x++) {
    osd_string[x] = 0;
  }
  osd_string[x] = 0xff;

  osd_attr = attributes;

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

// A VERY VERY STRIPED DOWN sprintf function suitable only for the paparazzi OSD.
static bool_t _osd_sprintf(char *buffer, char *string, float value)
{

  uint8_t param_start = 0;
  uint8_t param_end = 0;
  uint8_t frac_nb = 0;
  uint8_t digit = 0;
  uint8_t x = 0, y = 0, z = 0;

  uint16_t i_dec = 0;
  uint16_t i_frac = 0;

  char to_asc[10] = {48, 48, 48, 48, 48, 48, 48, 48, 48, 48};

  // Clear the osd string.
  for (x = 0; x < sizeof(osd_string); x++) {
    osd_string[x] = 0;
  }
  x = 0;
  // Search for the prameter start and stop positions.
  while (*(string + x) != '\0') {
    if (*(string + x) == '%') {
      param_start = x;
    } else if (*(string + x) == 'f') {
      param_end = x;
      break;
    }
    x++;
  }
  // find and bound the precision specified.
  frac_nb = *(string + param_end - 1) - 48; // Convert to number, ASCII 48 = '0'
  if (frac_nb > 3) {
    frac_nb = 3;       // Bound value.
  }
  y = (sizeof(to_asc) - 1); // Point y to the end of the array.
  i_dec = abs((int16_t)value);
  // Fist we will deal with the fractional part if specified.
  if (frac_nb > 0 && frac_nb <= 3) {
    i_frac = abs((int16_t)((value - (int16_t)value) * 1000)); // Max precision is 3 digits.
    x = 100;
    z = frac_nb;
    do {                            // Example if frac_nb=2 then 952 will show as .95
      z--;
      digit = (i_frac / x);
      to_asc[y + z] = digit + 48; // Convert to ASCII
      i_frac -= digit * x;      // Calculate the remainder.
      x /= 10;                  // 952-(9*100) = 52, 52-(10*5)=2 etc.
    } while (z > 0);
    y -= frac_nb;     // set y to point where the dot must be placed.
    to_asc[y] = '.';
    y--;             // Set y to point where the rest of the numbers must be written.
  }

  // Now it is time for the integer part. "y" already points to the position just before the dot.
  do {
    to_asc[y] = (i_dec % 10) + 48;            //Write at least one digit even if value is zero.
    i_dec /= 10;
    if (i_dec <= 0) {                          // This way the leading zero is ommited.
      if (value < 0) {
        y--; to_asc[y] = '-';  // Place the minus sign if needed.
      }
      break;
    } else {
      y--;
    }
  } while (1);

  // Fill the buffer with the characters in the beggining of the string if any.
  for (x = 0; x < param_start; x++) {
    *(buffer + x) = *(string + x);
  }

  // x is now pointing to the next character in osd_string.
  // y is already pointing to the first digit or negative sign in "to_asc" array.
  while (y < sizeof(to_asc)) {
    *(buffer + x) = to_asc[y];
    x++; y++;
  }
  // x is now pointing to the next character in osd_string.
  // "param_end" is pointing to the last format character in the string.
  do {
    param_end++;
    *(buffer + x++) = *(string + param_end);
  } while (*(string + param_end) != '\0'); //Write the rest of the string including the terminating char.

  return (0);
}
