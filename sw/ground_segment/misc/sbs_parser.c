/*
 * Copyright (C) 2013 Marc Schwarzbach
 *               2015 Felix Ruess <felix.ruess@gmail.com>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file sbs_parser.c
 * Parser for the SBS-1 protocol (ADS-B data).
 */

#include "sbs_parser.h"
#include <stdio.h>
#include <stdlib.h>

#define DEBUG_INPUT 0

#if DEBUG_INPUT == 1
#define INPUT_PRINT(...) { printf( __VA_ARGS__);};
#else
#define INPUT_PRINT(...) {  };
#endif


struct Intruder intruders[MAX_INTRUDERS];

static void parse_msg3(struct SbsMsgData *data);
static void parse_msg4(struct SbsMsgData *data);

static void mark_old_intruders(struct Intruder *intr);
static void sort_intruders(struct Intruder *intr);


/**
 * This is the actual parser.
 * It reads one character at a time
 * setting in_data.msg_available to TRUE
 * after a full line.
 */
void sbs_parse_char(struct SbsMsgData *data, unsigned char c)
{
  //reject empty lines
  if (data->msg_len == 0) {
    if (c == '\r' || c == '\n' || c == '$') {
      return;
    }
  }

  // fill the buffer, unless it's full
  if (data->msg_len < SBS_INPUT_MAXLEN - 1) {

    // messages end with a linefeed
    //AD: TRUNK:       if (c == '\r' || c == '\n')
    if (c == '\r' || c == '\n') {
      data->msg_available = 1;
    } else {
      data->msg_buf[data->msg_len] = c;
      data->msg_len++;
      data->msg_available = 0;
    }
  }

  if (data->msg_len >= SBS_INPUT_MAXLEN - 1) {
    data->msg_available = 1;
  }
}

/**
 * sbs_parse_char() has a complete line.
 * Find out what type of message it is and
 * hand it to the parser for that type.
MSG,4,,,495224,,,,,,,,404,54,,,64,,0,0,0,0
MSG,1,,,495224,,,,,,TAP594  ,,,,,,,,0,0,0,0
MSG,8,,,495224,,,,,,,,,,,,,,,,,
MSG,3,,,495224,,,,,,,37000,,,48.27750,11.68840,,,0,0,0,0
 */
void sbs_parse_msg(struct SbsMsgData *data)
{

  if (data->msg_len > 5 && !strncmp(data->msg_buf , "MSG,4", 5)) {
    data->msg_buf[data->msg_len] = 0;
    INPUT_PRINT("parsing MSG 4: \"%s\" \n\r", data->msg_buf);
    parse_msg4(data);
  } else {
    if (data->msg_len > 5 && !strncmp(data->msg_buf , "MSG,3", 5)) {
      data->msg_buf[data->msg_len] = 0;
      INPUT_PRINT("parsing MSG 3: \"%s\" \n\r", data->msg_buf);
      parse_msg3(data);
    } else {
      data->msg_buf[data->msg_len] = 0;
      INPUT_PRINT("ignoring: len=%i \"%s\" \n\r", data->msg_len, data->msg_buf);
    }
  }

  // reset message-buffer
  data->msg_len = 0;
}


static inline void read_until_sep(struct SbsMsgData *data, int *i)
{
  while (data->msg_buf[(*i)++] != ',') {
    if (*i >= data->msg_len) {
      return;
    }
  }
}


/**
 * parse MSG 4 SBS airborne velocity message
 *
 * MSG,4,0,1019,3C65A3,1119,2015/08/14,18:19:17.913,2015/08/14,18:19:17.913,,,511.6,48.6,,,128,,,,,
 */
static void parse_msg4(struct SbsMsgData *data)
{
  int i = 6;     // current position in the message, start after: MSG,4,
  char *endptr;  // end of parsed substrings

  // transmission type
  //int type = atoi(&data->msg_buf[4]);
  //if (type != 4) { return; }

  // aircraft ID
  read_until_sep(data, &i);
  int aircraft_id = atoi(&data->msg_buf[i]);
  INPUT_PRINT("MSG4 aircraft_id = %i \n\r", aircraft_id);

  // hex ident
  read_until_sep(data, &i);
  // Reading Hex number
  char hex_ident[10] = "foo";
  for (int j=0; j < 10; j++) {
    if (data->msg_buf[i+j] == ',') {
      hex_ident[j] = '\0';
      break;
    }
    hex_ident[j] = data->msg_buf[i+j];
  }
  hex_ident[9] = '\0';
  INPUT_PRINT("MSG4 hex_ident = %s\n\r", hex_ident);
  data->msg_buf[i - 2] = '0';
  data->msg_buf[i - 1] = 'x';
  //get Flarm intruder ID
  int __attribute__((unused)) flarm_id = strtod(&data->msg_buf[i - 2], &endptr);
  INPUT_PRINT("MSG4 flarm_id = %i \n\r", flarm_id);

  // flight ID
  read_until_sep(data, &i);
  int flight_id = atoi(&data->msg_buf[i]);
  INPUT_PRINT("MSG4 flight_id = %i\n\r", flight_id);

  // skip some fields
  // generated_date, generated_time, logged_date, logged_time, ?, ?
  int j;
  for (j = 0; j < 6; j++) {
    read_until_sep(data, &i);
  }

  //get intruder Ground speed (knots) and convert to m/s
  read_until_sep(data, &i);
  double speed = strtod(&data->msg_buf[i], &endptr) * 0.5144444;
  INPUT_PRINT("Speed = %f m/s\n\r", speed);

  //get intruder ground track
  read_until_sep(data, &i);
  double track = strtod(&data->msg_buf[i], &endptr);
  INPUT_PRINT("Track = %f deg\n\r", track);

  // next field: empty
  read_until_sep(data, &i);

  // next field: empty
  read_until_sep(data, &i);

  // next field: Climb rate
  read_until_sep(data, &i);

  //get intruder climb rate (feet per minute) and convert to m/s
  double climb = strtod(&data->msg_buf[i], &endptr) * 0.00508;
  INPUT_PRINT("Climb = %f m/s\n\r", climb);



  //Build table of current intruder situation

  //Check if already in list, then replace by new values
  int z ;
  int newflag = 1;

  for (z = 0; z < MAX_INTRUDERS; z++) {
    if (intruders[z].id == aircraft_id) {
      newflag = 0;
      //Check for positive or negative vertical speed (not signed :-( )
      if (intruders[z].lla.alt < intruders[z].lastalt) {
        //sinking
        climb = -climb;
      } else if (intruders[z].lla.alt == intruders[z].lastalt) {
        //equal.. take last
        if (intruders[z].climb < 0) {
          climb = -climb;
        }
      }

      intruders[z].gspeed = speed;
      intruders[z].course = track;
      intruders[z].climb  = climb;
      gettimeofday(&intruders[z].time4, NULL);
    }

  }

  //New intruder
  if (newflag) {
    intruders[MAX_INTRUDERS-1].id     = aircraft_id;
    intruders[MAX_INTRUDERS-1].flight_id = flight_id;
    strcpy(intruders[MAX_INTRUDERS-1].name, hex_ident);
    intruders[MAX_INTRUDERS-1].gspeed = speed;
    intruders[MAX_INTRUDERS-1].course = track;
    intruders[MAX_INTRUDERS-1].climb  = climb;
    intruders[MAX_INTRUDERS-1].used   = 4;
    gettimeofday(&intruders[MAX_INTRUDERS-1].time4, NULL);
    INPUT_PRINT("new = %i \n\r", intruders[MAX_INTRUDERS-1].id);
  }

  update_intruders(intruders);
}


/**
 * parse MSG 3 SBS Message (airborne position message)
 *
 * MSG,3,0,1019,3C65A3,1119,2015/08/14,18:18:21.714,2015/08/14,18:18:21.714,,35000,,,52.81230,13.39689,,,,0,0,0
 *
 * message_type
 * transmission_type
 * session_id
 * aircraft_id = parts[3];
 * hex_ident = parts[4];
 * flight_id = parts[5];
 * generated_date = parts[6];
 * generated_time = parts[7];
 * logged_date = parts[8];
 * logged_time = parts[9];
 * callsign = parts[10];
 * altitude = sbs1_value_to_int(parts[11]);
 * ground_speed = sbs1_value_to_int(parts[12]);
 * track = sbs1_value_to_int(parts[13]);
 * lat = sbs1_value_to_float(parts[14]);
 * lon = sbs1_value_to_float(parts[15]);
 * vertical_rate = sbs1_value_to_int(parts[16]);
 * squawk = parts[17];
 * alert = sbs1_value_to_bool(parts[18]);
 * emergency = sbs1_value_to_bool(parts[19]);
 * spi = sbs1_value_to_bool(parts[20]);
 * is_on_ground = sbs1_value_to_bool(parts[21]);
 */
static void parse_msg3(struct SbsMsgData *data)
{

  int i = 6;     // current position in the message, start after: MSG,3,
  char *endptr;  // end of parsed substrings
  struct LlaCoor_d lla;

  // transmission type
  //int type = atoi(&data->msg_buf[4]);
  //if (type != 3) { return; }

  // aircraft ID
  read_until_sep(data, &i);
  int aircraft_id = atoi(&data->msg_buf[i]);
  INPUT_PRINT("MSG3 aircraft_id = %i \n\r", aircraft_id);

  // hex ident
  read_until_sep(data, &i);
  //Reading Hex number
  char hex_ident[10] = "foo";
  for (int j=0; j < 10; j++) {
    if (data->msg_buf[i+j] == ',') {
      hex_ident[j] = '\0';
      break;
    }
    hex_ident[j] = data->msg_buf[i+j];
  }
  hex_ident[9] = '\0';
  INPUT_PRINT("MSG3 hex_ident = %s\n\r", hex_ident);
  data->msg_buf[i - 2] = '0';
  data->msg_buf[i - 1] = 'x';
  //get Flarm intruder ID
  int __attribute__((unused)) flarm_id = strtod(&data->msg_buf[i - 2], &endptr);
  INPUT_PRINT("MSG3 flarm_id = %i \n\r", flarm_id);

  // flight ID
  read_until_sep(data, &i);
  int flight_id = atoi(&data->msg_buf[i]);
  INPUT_PRINT("MSG3 flight_id = %i\n\r", flight_id);

  // skip some fields
  // generated_date, generated_time, logged_date, logged_time, callsign
  int j;
  for (j = 0; j < 5; j++) {
    read_until_sep(data, &i);
  }

  //get intruder alt (in feet)
  read_until_sep(data, &i);
  lla.alt = strtod(&data->msg_buf[i], &endptr) * 0.3048; //feet to m
  INPUT_PRINT("Alt = %f m\n\r", lla.alt);

  // ground_speed
  read_until_sep(data, &i);

  // track
  read_until_sep(data, &i);

  // latitude
  read_until_sep(data, &i);
  double lat = strtod(&data->msg_buf[i], &endptr);
  INPUT_PRINT("Lat = %f deg\n\r", lat);
  lla.lat = RadOfDeg(lat);

  // longitude
  read_until_sep(data, &i);
  double lon = strtod(&data->msg_buf[i], &endptr);
  INPUT_PRINT("Lon = %f deg\n\r", lon);
  lla.lon = RadOfDeg(lon);

  // vertical rate
  read_until_sep(data, &i);

  // not using the rest of the fields atm


  //Build table of current intruder situation

  //Check if already in list, then replace by new values
  int z ;
  int newflag = 1;

  for (z = 0; z < MAX_INTRUDERS; z++) {
    if (intruders[z].id == aircraft_id) {
      newflag = 0;
      intruders[z].lastalt  = intruders[z].lla.alt;
      intruders[z].lla      = lla;
      gettimeofday(&intruders[z].time3, NULL);
    }

  }

  //New intruder
  //If Relative East is empty (value 0), it is a mode C/S transponder, so no position.
  if (newflag) {
    intruders[MAX_INTRUDERS-1].id       = aircraft_id;
    intruders[MAX_INTRUDERS-1].flight_id = flight_id;
    strcpy(intruders[MAX_INTRUDERS-1].name, hex_ident);
    intruders[MAX_INTRUDERS-1].lla      = lla;
    intruders[MAX_INTRUDERS-1].used     = 3;
    gettimeofday(&intruders[MAX_INTRUDERS-1].time3, NULL);
    INPUT_PRINT("new = %i \n\r", intruders[MAX_INTRUDERS-1].id);
  }

  update_intruders(intruders);
}

void update_intruders(struct Intruder *intr)
{
  mark_old_intruders(intr);
  sort_intruders(intr);
}


/** elapsed time in milliseconds between two timevals */
static inline unsigned int time_elapsed_ms(struct timeval *prev, struct timeval *now)
{
  time_t d_sec = now->tv_sec - prev->tv_sec;
  long d_usec = now->tv_usec - prev->tv_usec;
  /* wrap if negative microseconds */
  if (d_usec < 0) {
    d_sec -= 1;
    d_usec += 1000000L;
  }
  return d_sec * 1000 + d_usec / 1000;
}


#define MAX_AGE_INTR 5000   ///< max age in ms before intruder is marked as unused

/// mark intruders as unused if data too old
static void mark_old_intruders(struct Intruder *intr)
{

  struct timeval now;
  gettimeofday(&now, NULL);

  //Analyse if Data ok
  for (int i = 0; i < MAX_INTRUDERS; i++) {
    //If data too old, mark unused
    if ((time_elapsed_ms(&intr[i].time3, &now) < MAX_AGE_INTR) &&
        (time_elapsed_ms(&intr[i].time4, &now) < MAX_AGE_INTR))
    {
      intr[i].used = 1 ;
    } else if (time_elapsed_ms(&intr[i].time3, &now) < MAX_AGE_INTR) {
      intr[i].used = 3 ;
    } else if (time_elapsed_ms(&intr[i].time4, &now) < MAX_AGE_INTR) {
      intr[i].used = 4 ;
    } else {
      intr[i].used = 0;
    }
  }

  for (int i = 0; i < MAX_INTRUDERS; i++) {
    INPUT_PRINT("Nr:%i u:%d\n", i, intr[i].used);
  }
  INPUT_PRINT("fin \n\r");
}


#define timercmp(a, b, CMP)          \
  (((a)->tv_sec == (b)->tv_sec) ?    \
   ((a)->tv_usec CMP (b)->tv_usec) : \
   ((a)->tv_sec CMP (b)->tv_sec))


/// Sort intruder array for used and last updated
static void sort_intruders(struct Intruder *intr)
{
  //Sort for used and last updated
  struct Intruder temp_int;
  //Bubble sort
  for (int i = 0; i < MAX_INTRUDERS-1; i++) {
    for (int j = 0; j < MAX_INTRUDERS-1 - i; j++) {
      if (((intr[j].used == 0) && (intr[j + 1].used != 0)) ||
          (timercmp(&intr[j].time4, &intr[j + 1].time4, >) && (intr[j + 1].used == 1)))
      {
        temp_int = intr[j];
        intr[j] = intr[j + 1];
        intr[j + 1] = temp_int;
      }
    }
  }
}
