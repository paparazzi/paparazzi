/*
 * $Id$
 *  
 * Copyright (C) 2009 ENAC
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

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "gsm.h"
#include "uart.h"
#include "std.h"
#include "ap_downlink.h"
#include "gps.h"
#include "autopilot.h"
#include "estimator.h"
#include "common_nav.h"

#ifndef GSM_LINK
#define GSM_LINK Uart3100
#endif

#define GSM_MAX_PAYLOAD 160

#define __GSMLink(dev, _x) dev##_x
#define _GSMLink(dev, _x)  __GSMLink(dev, _x)
#define GSMLink(_x) _GSMLink(GSM_LINK, _x)

#define GSMBuffer() GSMLink(ChAvailable())
#define ReadGSMBuffer() { while (GSMLink(ChAvailable())&&!gsm_line_received) gsm_parse(GSMLink(Getch())); }

#define GSMTransmit(_c) GSMLink(Transmit(_c))

#define CTRLZ 		0x1A
#define GSM_ORIGIN_MAXLEN 32
#define DATA_MAXLEN 128

static bool gsm_line_received;
static bool prompt_received;
static bool wait_data; /* Waiting for message content after header */
static bool is_receiving; /* +CMTI has been received, true until contents has been parsed */
static bool waiting_for_reply; /* An AT command has been sent and an answer is expected */ 
static bool init_done;
static bool waiting_prompt;
static bool msg_sent;

static char msg_status[16];
static char msg_date[32];

static char expected_ack[10];
static char gsm_buf[GSM_MAX_PAYLOAD] __attribute__ ((aligned));
static uint8_t gsm_buf_idx, gsm_buf_len;
static char origin[GSM_ORIGIN_MAXLEN];
static char data_to_send[DATA_MAXLEN];

#define STATUS_NONE           0
#define STATUS_CSQ            1
#define STATUS_REQUESTING_MESSAGE 2
#define STATUS_SEND_AT	      3
#define STATUS_SEND_CMGF      4
#define STATUS_SEND_CNMI      5
#define STATUS_SEND_CPMS      6
#define STATUS_RECEPTION_SMS2 7
static uint8_t gsm_status = STATUS_NONE;

static uint8_t nb_received_lines = 0;
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
static void Extraction(char source[], char char_dbt, int nb_occurence1, int decalage1, char char_fin, int nb_occurence2, int decalage2, char destination[]);
static void Send_CSQ(void);
static void receiving_header(void);
static void Send(const char string[]);
static void parse_msg_header(char* reponse_module);





/*****************************************************************************/
void gsm_init(void) {
  gsm_buf_idx = 0;
  gsm_line_received = false;
  init_done = false;
  waiting_prompt = false;
  msg_sent = true;

  Send_AT();
}


void gsm_event(void) {
  if (GSMBuffer())
    ReadGSMBuffer();

  if (gsm_line_received) {
    gsm_got_line();
    gsm_line_received = false;
  } else if (prompt_received) {
    gsm_got_prompt();
    prompt_received = false;
  }
}


// A line of length gsm_buf_len is available in the gsm_buf buffer
static void gsm_got_line(void)
{
  if (wait_data) { // Currently receiving a SMS
    gsm_receive_content();
  } else if ((strncmp("+CMTI:", gsm_buf, strlen("+CMTI")) == 0)
	     && (!is_receiving)) { // A SMS is available
    is_receiving = true;
    // ???? Send(gsm_buf);
 
    char index_extrait[6] = "\0\0\0\0\0\0";     
      
    /* Extraction de l'index du message recu */
    Extraction(gsm_buf, ',', 1, 1, '\r', 1, 1, index_extrait);

    //??? Send(index_extrait);
      
    index_msg = atoi(index_extrait);
    
    request_for_msg();
  } else if (waiting_for_reply) { // Other cases
    // Do we get what we were expecting
    
    bool GSM_reponse = strncmp(expected_ack, gsm_buf, strlen(expected_ack)) == 0;
    switch(gsm_status) {
    case STATUS_CSQ :			
      if(!GSM_reponse) {
	if(nb_received_lines <= 15) { // ????? Telit is sending empty lines ?
	  nb_received_lines++;
	  waiting_for_reply = true; // Waiting longer for an answer
	} else {
	  nb_received_lines = 0;
	  Send_CSQ(); // Asking again for the network coverage
	}
      } else {
	waiting_for_reply = false;
	gsm_send_report_continue();
      }
      break;
      
    case STATUS_REQUESTING_MESSAGE :
      if(!GSM_reponse) {
	if(nb_received_lines <= 10) {
	  nb_received_lines++;
	  waiting_for_reply = true;
	} else {
	  nb_received_lines = 0;
	  request_for_msg();
	}
      } else {
	waiting_for_reply = false;
	receiving_header();
      }
      break;
      
    case STATUS_SEND_AT :
      if(!GSM_reponse) {
	if(nb_received_lines <= 10) {
	  nb_received_lines++;
	  waiting_for_reply = true;
	} else {
	  nb_received_lines = 0;
	  waiting_for_reply = true;
	  Send_AT();
	}
      } else {
	waiting_for_reply = false;
	GSM_reponse = false;
	Send_CMGF();
      }
      break;
      
    case STATUS_SEND_CMGF :
      if(!GSM_reponse) {
	if(nb_received_lines <= 10) {
	  nb_received_lines++;
	  waiting_for_reply = true;
	} else {
	  nb_received_lines = 0;
	  waiting_for_reply = true;
	  Send_CMGF();
	}
      } else {
	waiting_for_reply = false;
	GSM_reponse = false;
	Send_CNMI();
      }
      break;
      
    case STATUS_SEND_CNMI :
      if(!GSM_reponse) {
	if(nb_received_lines <= 10) {
	  nb_received_lines++;
	  waiting_for_reply = true;
	} else {
	  nb_received_lines = 0;
	  waiting_for_reply = true;
	  Send_CNMI();
	}
      } else {
	waiting_for_reply = false;
	GSM_reponse = false;
	Send_CPMS();
      }
      break;
      
    case STATUS_SEND_CPMS :
      if(!GSM_reponse) {
	if(nb_received_lines <= 10) {
	  nb_received_lines++;
	  waiting_for_reply = true;
	} else {
	  nb_received_lines = 0;
	  waiting_for_reply = true;
	  Send_CPMS();
	}
      } else {
	GSM_reponse = false;
	init_done = true;
	waiting_for_reply = false;
      }								
      break;
      
    default:				
      break;
    }
  }
}



// Receiving a SMS, first step: asking for a given message
static void request_for_msg(void)
{
  char demande_lecture_SMS[10];
  
  strcpy(expected_ack, "+CMGR");
  sprintf(demande_lecture_SMS, "AT+CMGR=%d", index_msg);
  gsm_status = STATUS_REQUESTING_MESSAGE;
  waiting_for_reply = true;
  Send(demande_lecture_SMS);
}


// Receiving a SMS, second step, header available in gsm_buf
static void receiving_header(void)
{
  parse_msg_header(gsm_buf);
  wait_data = true;
}

// Receiving a SMS, third step, content in gsm_buf
static void gsm_receive_content(void)
{
  char instruction[15], index_block[2];
		
  wait_data = false;

  // ?????? sprintf(data_to_send, "%d %s %s %s %s", index_msg, flag, expediteur, dateheure, data_recue);
  // ?????? Send(data_to_send);
  
  // Checking the number of the sender
  if (strncmp((char*)GCS_NUMBER, origin, 12) == 0) {
      // Decoding the message ...
	
      // Search for the instruction
      Extraction(gsm_buf, ' ', 1, 1, ' ', 1, 0, instruction);
	
      Send(instruction);
	
      // Si on a recu une instruction JUMP_TO_BLOCK	
      if (strcmp(instruction, "JUMP_TO_BLOCK") == 0) {
	Extraction(gsm_buf, ' ', 2, 1, '\n', 1, 0, index_block);
	
	nav_goto_block(atoi(index_block));
      }
	
      Suppr_SMS(index_msg);
  }
  
  is_receiving = false;
}


// Deleting a SMS
void Suppr_SMS(int index_)
{
  char demande_suppression[20];
	
  sprintf(demande_suppression, "AT+CMGD=%d", index_);
  strcpy(expected_ack, "OK");
  Send(demande_suppression);
}


// Sending a SMS, first step: send the number and wait for the prompt
static void Send_Msg(void)
{
  waiting_prompt = true;
  
  Send("AT+CMGS=\"+33617802096\"\0");
}

// We just have received a prompt ">" (we are sending a SMS)
static void gsm_got_prompt(void)
{
  if (waiting_prompt) { // We were waiting for a prompt
    char string[strlen(data_to_send) +3];
    
    sprintf(string, "%s%c", data_to_send, CTRLZ);
    Send(string);
  }
  
  waiting_prompt = false;
}


static void parse_msg_header(char* buffer2)
{
  /* Extraction du flag*/
  Extraction(buffer2, '"', 1, 1, '"', 1, 0, msg_status);
	
  /* Extraction de l'expediteur*/
  Extraction(buffer2, '"', 2, 1, '"', 1, 0, origin);
	
  /* Extraction de date heure*/
  Extraction(buffer2, '"', 4, 1, '"', 1, 0, msg_date);
	
  //pb d'ecriture du flag => solution de fortune (pb si flag != rec unread)
  //??????? strncpy(flag, flag, 10);
}



// Periodic message, first step (called every 60s)
void gsm_send_report()
{
  if(init_done && msg_sent) {
    msg_sent = false;
    
    // Checking the network coverage
    Send_CSQ();
  } 
}


// Sending a message, second step; we have asked for network quality
void gsm_send_report_continue(void)
{
  char qualite_signal_GSM[5];
  
  //We got "+CSQ:" (and we expect "ok" on the second line ???)
  Extraction(gsm_buf, ':', 1, 2, '\r', 1, 0, qualite_signal_GSM);
  
  qualite_signal_GSM[4] = '\0';	  
	
  // Donnee GPS :ne sont pas envoyes gps_mode, gps_itow, gps_utm_zone, gps_nb_ovrn
  // Donnees batterie (seuls vsupply et estimator_flight_time sont envoyes)
  // concatenation de toutes les infos en un seul message à transmettre
  sprintf(data_to_send, "%ld %ld %d %ld %d %d %d %d %s", gps_utm_east, gps_utm_north, gps_course, gps_alt, gps_gspeed, gps_climb, vsupply, estimator_flight_time, qualite_signal_GSM);
  
  // Envoi du SMS
  Send_Msg();
  
  msg_sent = true;
}




static void Send_AT(void)
{
  strcpy(expected_ack, "OK");
  gsm_status = STATUS_SEND_AT;
  waiting_for_reply = true;
 
  Send("AT");
}

static void Send_CMGF(void)
{ 
  strcpy(expected_ack, "OK");
  gsm_status = STATUS_SEND_CMGF;
  waiting_for_reply = true;
  Send("AT+CMGF=1");
}

static void Send_CSQ(void)
{ 
  strcpy(expected_ack, "OK");
  gsm_status = STATUS_CSQ;
  waiting_for_reply = true;
  Send("AT+CSQ");
}

static void Send_CNMI(void)
{ 
  strcpy(expected_ack, "OK");
  gsm_status = STATUS_SEND_CNMI;
  waiting_for_reply = true;
  Send("AT+CNMI=1,1,0,0,0");
}

static void Send_CPMS(void)
{ 
  strcpy(expected_ack, "+CPMS:");
  gsm_status = STATUS_SEND_CPMS;
  waiting_for_reply = true;
  Send("AT+CPMS=\"SM\"");
}


static uint8_t indexn(const char* string, char c, int nb_occ)
{
  uint8_t nb_trouve = 0, i = 0;
	
  while(string[i] && nb_trouve < nb_occ) {
    if(string[i] == c) {
      nb_trouve++;
    }
    i++;
  }
    
  if (nb_trouve == nb_occ)
    return i-1;
  else
    return -1;
}



static void gsm_parse(uint8_t c) {
  switch(c) {
  case '\n':
    /* ignore */
    break;
  case '\r':
    gsm_line_received = true;
    gsm_buf_len = gsm_buf_idx;
    gsm_buf_idx=0;
    break;
  case '>':
    prompt_received = true;
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

  while(string[i])
    GSMTransmit(string[i]);
  GSMTransmit('\r');
}

static void Extraction(char source[], char char_dbt, int nb_occurence1, int decalage1, char char_fin, int nb_occurence2, int decalage2, char destination[])
{
  int indice_debut, indice_fin, i, rch1, rch2;
  
  rch1 = indexn(source, char_dbt, nb_occurence1);
  
  if(rch1 == -1)
    strncpy(destination, "0", 1);
  else {
    indice_debut = rch1 + decalage1;
    for(i=0; i<strlen(source); i++)
      source[i] = source[i+indice_debut];
    
    rch2 = indexn(source, char_fin, nb_occurence2);
    
    if(rch2 == -1)
      strncpy(destination, "0", 1);
    else {
      indice_fin = rch2 + decalage2;
      strncpy(destination, source, indice_fin);
      destination[indice_fin] = '\0';
    }
  }
}
