/*
 * Copyright (C) 2009 ENAC, Arnaud Quintard, Pascal Brisset
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

/*
http://www.telit.com/en/products/gsm-gprs.php?p_ac=show&p=12#downloads

Init:
  Out: ATE0
  In: OK
  Out: AT+CMGF=1
  In: OK
  Out: AT+CNMI=1,1,0,0,0
  In: OK
  Out : AT+CPMS=\"SM\"
  In: +CPMS:

Reporting:
  Out: AT+CSQ
  In: +CSQ: <rssi>,<ber>
  In: OK
  Out: AT+CMGS=\"GCS_NUMBER\"
  In: >
  Out: gps.utm_pos.east, gps.utm_pos.north, gps.course, gps.hmsl, gps.gspeed, -gps.ned_vel.z, electrical.vsupply, autopilot.flight_time, rssi  CTRLZ

Receiving:
  In: +CMTI: ...,<number>
  Out: AT+CMGR=<number>
  In: +CMGR ...
  In: B42 (or S42 3.14)
  Out: AT+CMGD=<number>
  In: OK
*/

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "gsm.h"
#include "mcu_periph/uart.h"
#include "std.h"
#include "subsystems/datalink/downlink.h"
#include "subsystems/gps.h"
#include "autopilot.h"
#include "subsystems/electrical.h"
//#include "subsystems/navigation/common_nav.h"  //why is should this be needed?
#include "generated/settings.h"
#include "led.h"

#ifndef GSM_LINK
#define GSM_LINK UART3100
#endif

#define GSM_MAX_PAYLOAD 160

#define GSMLinkDev (&(GSM_LINK).device)

#define GSMLinkChAvailable() GSMLinkDev->check_available(GSMLinkDev->periph)
#define GSMLinkTransmit(_c) GSMLinkDev->put_byte(GSMLinkDev->periph, 0, _c)
#define GSMLinkGetch() GSMLinkDev->get_byte(GSMLinkDev->periph)
#define ReadGSMBuffer() { while (GSMLinkChAvailable&&!gsm_line_received) gsm_parse(GSMLinkGetch()); }


#define CTRLZ     0x1A
#define GSM_ORIGIN_MAXLEN 32
#define DATA_MAXLEN 128

#define CMTI "+CMTI:"
#define MAXLEN_CMTI_ANSWER 32
#define MAXLEN_SMS_CONTENT DATA_MAXLEN

static bool gsm_line_received;
static bool prompt_received;
static bool waiting_for_reply; /* An AT command has been sent and an answer is expected */

// static char msg_status[16];
// static char msg_date[32];

static char expected_ack[10];
static char gsm_buf[GSM_MAX_PAYLOAD] __attribute__((aligned));
static uint8_t gsm_buf_idx, gsm_buf_len;
static char origin[GSM_ORIGIN_MAXLEN];
static char data_to_send[DATA_MAXLEN];

#define STATUS_NONE           0
#define STATUS_CSQ            1
#define STATUS_REQUESTING_MESSAGE 2
#define STATUS_SEND_AT        3
#define STATUS_SEND_CMGF      4
#define STATUS_SEND_CNMI      5
#define STATUS_SEND_CPMS      6
#define STATUS_RECEPTION_SMS2 7
#define STATUS_WAITING_DATA   8
#define STATUS_IDLE           9
#define STATUS_WAITING_PROMPT 10
#define STATUS_DELETE_SMS     11
#define STATUS_POWERON        12

static uint8_t gsm_status = STATUS_NONE;

static uint8_t index_msg;

static void Send_AT(void);
static void Send_CMGF(void);
static void Send_CNMI(void);
static void Send_CPMS(void);
static void Suppr_SMS(int);
static void gsm_send_report_continue(void);
static void gsm_parse(uint8_t c);
static void gsm_got_line(void);
static void gsm_got_prompt(void);
static void gsm_receive_content(void);
static void request_for_msg(void);
static void Send_CSQ(void);
static void Send(const char string[]);
static void parse_msg_header(void);
static char *indexn(char *, char, uint8_t);


static uint8_t gcs_index;
static uint8_t gcs_index_max;


/*****************************************************************************/
void gsm_init(void)
{
  if (gsm_status == STATUS_NONE) { /* First call */
    LED_ON(GSM_ONOFF_LED);
    gsm_status = STATUS_POWERON;
    //} else { /* Second call */
    //  gsm_buf_idx = 0;
    //  gsm_line_received = false;
    //
    //  Send_AT();
    //  gsm_status = STATUS_SEND_AT;
    //  gsm_gsm_init_status = false;
  }
  gcs_index = 0;
  gcs_index_max = 0;
#ifdef GCS_NUMBER_1
  gcs_index_max++;
#endif
#ifdef GCS_NUMBER_2
  gcs_index_max++;
#endif
}

void gsm_init_report(void)   /* Second call */
{
  if (gsm_status != STATUS_NONE) {
    gsm_buf_idx = 0;
    gsm_line_received = false;

    Send_AT();
    gsm_status = STATUS_SEND_AT;
    gsm_gsm_init_report_status = false;
  }
}

void gsm_event(void)
{
  if (GSMLinkChAvailable()) {
    ReadGSMBuffer();
  }

  if (gsm_line_received) {
    if (gsm_buf_len > 0) { DOWNLINK_SEND_DEBUG_GSM_RECEIVE(DefaultChannel, DefaultDevice, gsm_buf_len, gsm_buf); }
    gsm_got_line();
    gsm_line_received = false;
  } else if (prompt_received) {
    DOWNLINK_SEND_DEBUG_GSM_RECEIVE(DefaultChannel, DefaultDevice, 1, ">");
    gsm_got_prompt();
    prompt_received = false;
  }
}


// A line of length gsm_buf_len is available in the gsm_buf buffer
static void gsm_got_line(void)
{
  if (gsm_status == STATUS_WAITING_DATA) { // Currently receiving a SMS
    gsm_receive_content();
    Suppr_SMS(index_msg);
    gsm_status = STATUS_DELETE_SMS;
  } else if (gsm_status == STATUS_IDLE
             && strncmp(CMTI, gsm_buf, strlen(CMTI)) == 0) {
    /* A SMS is available */
    /* Extracting the index of the message */
    char *first_comma = indexn(gsm_buf, ',', MAXLEN_CMTI_ANSWER);
    if (first_comma) {
      index_msg = atoi(first_comma + 1);
      request_for_msg();
      gsm_status = STATUS_REQUESTING_MESSAGE;
    }
  } else if (waiting_for_reply) { // Other cases
    // Do we get what we were expecting

    bool gsm_answer = strncmp(expected_ack, gsm_buf, strlen(expected_ack)) == 0;
    if (gsm_answer) {
      waiting_for_reply = false;

      switch (gsm_status) {
        case STATUS_CSQ :
          gsm_send_report_continue();
          gsm_status = STATUS_WAITING_PROMPT;
          break;

        case STATUS_REQUESTING_MESSAGE:
          parse_msg_header();
          gsm_status = STATUS_WAITING_DATA;
          break;

        case STATUS_SEND_AT :
          gsm_answer = false;
          Send_CMGF();
          gsm_status = STATUS_SEND_CMGF;
          break;

        case STATUS_SEND_CMGF :
          gsm_answer = false;
          Send_CNMI();
          gsm_status = STATUS_SEND_CNMI;
          break;

        case STATUS_SEND_CNMI :
          gsm_answer = false;
          Send_CPMS();
          gsm_status = STATUS_SEND_CPMS;
          break;

        case STATUS_SEND_CPMS :
          gsm_answer = false;
          gsm_status = STATUS_IDLE;
          gsm_gsm_send_report_status = MODULES_START; /** Start reporting */
          break;

        case STATUS_DELETE_SMS :
          gsm_status = STATUS_IDLE;
          break;

        default:
          break;
      }
    } else { /** We did not get the expected answer */
      /* Let's wait for the next line */
    }
  }
}



// Receiving a SMS, first step: asking for a given message
static void request_for_msg(void)
{
  char demande_lecture_SMS[16];

  strcpy(expected_ack, "+CMGR");
  sprintf(demande_lecture_SMS, "AT+CMGR=%d", index_msg);
  waiting_for_reply = true;
  Send(demande_lecture_SMS);
}


/** Receiving a SMS, third step, content in gsm_buf
  Message can be
  Bdd         where dd is a block index on two digits. WARNING: dd > 0
  Sdd value   where dd>0 is a var index on two digits and value is a float
  */
static void gsm_receive_content(void)
{
  // ?????? sprintf(data_to_send, "%d %s %s %s %s", index_msg, flag, expediteur, dateheure, data_recue);
  // ?????? Send(data_to_send);

  // Checking the number of the sender
  if (
//#if ! (defined GCS_NUMBER_1 || defined GCS_NUMBER_2 || defined SAFETY_NUMBER_1 || defined SAFETY_NUMBER_2)
    true
//#else
//      false
//#endif
#ifdef GCS_NUMBER_1
    || strncmp((char *)GCS_NUMBER_1, origin, strlen(GCS_NUMBER_1)) == 0
#endif
#ifdef GCS_NUMBER_2
    || strncmp((char *)GCS_NUMBER_2, origin, strlen(GCS_NUMBER_2)) == 0
#endif
#ifdef SAFETY_NUMBER_1
    || strncmp((char *)SAFETY_NUMBER_1, origin, strlen(SAFETY_NUMBER_1)) == 0
#endif
#ifdef SAFETY_NUMBER_2
    || strncmp((char *)SAFETY_NUMBER_2, origin, strlen(SAFETY_NUMBER_2)) == 0
#endif
  ) {
    // Decoding the message ...

    // Search for the instruction
    switch (gsm_buf[0]) {
      case 'B' : {
        uint8_t block_index = atoi(gsm_buf + 1);
        if (block_index > 0) { /* Warning: no way to go to the first block */
          nav_goto_block(block_index);
        }
        break;
      }
      case 'S' : {
        uint8_t var_index = atoi(gsm_buf + 1);
        if (var_index > 0) {
          float value = atof(indexn(gsm_buf, ' ', MAXLEN_SMS_CONTENT) + 1);
          DlSetting(var_index, value);
        }
      }

      default:
        // Report an error ???
        break;
    }
  }
}


// Deleting a SMS
void Suppr_SMS(int index_)
{
  char demande_suppression[20];

  sprintf(demande_suppression, "AT+CMGD=%d", index_);
  strcpy(expected_ack, "OK");
  waiting_for_reply = true;
  Send(demande_suppression);
}


// We just have received a prompt ">" (we are sending a SMS)
static void gsm_got_prompt(void)
{
  if (gsm_status == STATUS_WAITING_PROMPT) { // We were waiting for a prompt
    char string[strlen(data_to_send) + 3];

    sprintf(string, "%s%c", data_to_send, CTRLZ);
    Send(string);
  }

  gsm_status = STATUS_IDLE;
}

/** Message header in gsm_bug
 */
static void parse_msg_header(void)
{
  /* Extraction du flag*/
  /** Extraction(buffer2, '"', 1, 1, '"', 1, 0, msg_status); */

  /* Extraction de l'expediteur*/
  // Extraction(buffer2, '"', 2, 1, '"', 1, 0, origin);

  /* Extraction de date heure*/
  // Extraction(buffer2, '"', 4, 1, '"', 1, 0, msg_date);

  //pb d'ecriture du flag => solution de fortune (pb si flag != rec unread)
  //??????? strncpy(flag, flag, 10);
}



// Periodic message, first step (called every 60s)
void gsm_send_report()
{
  gsm_status = STATUS_IDLE;
  if (gsm_status == STATUS_IDLE) {
    // Checking the network coverage
    Send_CSQ();
    gsm_status = STATUS_CSQ;
  }
}


// Sending a message, second step; we have asked for network quality
void gsm_send_report_continue(void)
{
  //We got "+CSQ: <rssi>,<ber>" <rssi> and <ber> on 2 digits (cf 3.5.4.4.4)
  // and we expect "OK" on the second line
  uint8_t rssi = atoi(gsm_buf + strlen("+CSQ: "));

  // Donnee GPS :ne sont pas envoyes gps_mode, gps.tow, gps.utm_pos.zone, gps_nb_ovrn
  // Donnees batterie (seuls vsupply et autopilot.flight_time sont envoyes)
  // concatenation de toutes les infos en un seul message à transmettre
  sprintf(data_to_send, "%ld %ld %d %ld %d %d %d %d %d", gps.utm_pos.east, gps.utm_pos.north, gps_course, gps.hmsl,
          gps.gspeed, -gps.ned_vel.z, electrical.vsupply, autopilot.flight_time, rssi);

  // send the number and wait for the prompt
  char buf[32];
  switch (gcs_index) {
#ifdef GCS_NUMBER_1
    case 0 :
      sprintf(buf, "AT+CMGS=\"%s\"", GCS_NUMBER_1);
      Send(buf);
      break;
#endif
#ifdef GCS_NUMBER_2
    case 1 :
      sprintf(buf, "AT+CMGS=\"%s\"", GCS_NUMBER_2);
      Send(buf);
      break;
#endif
    default :
      gcs_index = 0;
      break;
  }
  gcs_index++;
  if (gcs_index == gcs_index_max) { gcs_index = 0; }
}




static void Send_AT(void)
{
  strcpy(expected_ack, "OK");
  waiting_for_reply = true;

  Send("ATE0");
}

static void Send_CMGF(void)
{
  strcpy(expected_ack, "OK");
  waiting_for_reply = true;
  Send("AT+CMGF=1");
}

static void Send_CSQ(void)
{
  /***** FIXME ******  strcpy(expected_ack, "+CSQ:"); ****/
  strcpy(expected_ack, "OK");
  waiting_for_reply = true;
  Send("AT+CSQ");
}

static void Send_CNMI(void)
{
  strcpy(expected_ack, "OK");
  waiting_for_reply = true;
  Send("AT+CNMI=1,1,0,0,0");
}

static void Send_CPMS(void)
{
  strcpy(expected_ack, "+CPMS:");
  waiting_for_reply = true;
  Send("AT+CPMS=\"SM\"");
}


static void gsm_parse(uint8_t c)
{
  switch (c) {
    case GSM_CMD_LINE_TERMINATION:
      break;
    case '>':
      prompt_received = true;
      break;
    case GSM_RESPONSE_FORMATING:
      gsm_buf[gsm_buf_idx] = '\0';
      gsm_line_received = true;
      gsm_buf_len = gsm_buf_idx;
      gsm_buf_idx = 0;
      break;
    default:
      if (gsm_buf_idx < GSM_MAX_PAYLOAD) {
        gsm_buf[gsm_buf_idx] = c;
        gsm_buf_idx++;
      } /* else extra characters are ignored */
      break;
  }
}


// Sending a string to the GSM module (through the UART)
static void Send(const char string[])
{
  int i = 0;

  while (string[i]) {
    GSMTransmit(string[i++]);
  }
  GSMTransmit(GSM_CMD_LINE_TERMINATION);

  DOWNLINK_SEND_DEBUG_GSM_SEND(DefaultChannel, DefaultDevice, i, string);
}

/* Returns a pointer to the first occurrence of the character c in the firtn
   n chars of string s. Return NULL if not found */
static char *indexn(char *s, char c, uint8_t n)
{
  while (n && (*s != c)) {
    n--;
    s++;
  }
  return (n ? s : NULL);
}
