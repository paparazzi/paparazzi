/*
 * Paparazzi mcu0 $Id$
 *  
 * Copyright (C) 2005  Pascal Brisset, Antoine Drouin
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

/* Coronis wavecard serial input and output */

#ifndef WAVECARD_H
#define WAVECARD_H

#include "uart.h"

#ifdef WAVECARD_ON_GPS
#define WcPut1CtlByte(x) uart1_transmit(x)
#else
#define WcPut1CtlByte(x) uart0_transmit(x)
#endif

#define g_message(_)

#define WC_CTL_BYTE_LEN 4
#define WC_ADDR_LEN     6


extern uint8_t wc_msg_received;

extern uint8_t  wc_length;
void wc_parse_payload(void);



#define WC_SYNC 0xff
#define WC_STX 0x02
#define WC_ETX 0x03


#define WC_ACK 0x06
#define WC_NAK 0x15
#define WC_ERROR 0x00
#define WC_REQ_WRITE_RADIO_PARAM 0x40
#define WC_RES_WRITE_RADIO_PARAM 0x41
#define WC_REQ_READ_RADIO_PARAM 0x50
#define WC_RES_READ_RADIO_PARAM 0x51
#define WC_REQ_SELECT_CHANNEL 0x60
#define WC_RES_SELECT_CHANNEL 0x61
#define WC_REQ_READ_CHANNEL 0x62
#define WC_RES_READ_CHANNEL 0x63
#define WC_REQ_SELECT_PHYCONFIG 0x64
#define WC_RES_SELECT_PHYCONFIG 0x65
#define WC_REQ_READ_PHYCONFIG 0x66
#define WC_RES_READ_PHYCONFIG 0x67
#define WC_REQ_READ_REMOTE_RSSI 0x68
#define WC_RES_READ_REMOTE_RSSI 0x69
#define WC_REQ_READ_LOCAL_RSSI 0x6A
#define WC_RES_READ_LOCAL_RSSI 0x6B
#define WC_REQ_FIRMWARE_VERSION 0xA0
#define WC_RES_FIRMWARE_VERSION 0xA1
#define WC_MODE_TEST 0xB0
#define WC_REQ_SEND_FRAME 0x20
#define WC_RES_SEND_FRAME 0x21
#define WC_REQ_SEND_MESSAGE 0x22
#define WC_REQ_SEND_POLLING 0x26
#define WC_REQ_SEND_BROADCAST 0x28
#define WC_RECEIVED_FRAME 0x30
#define WC_RECEPTION_ERROR 0x31
#define WC_RECEIVED_FRAME_POLLING 0x32
#define WC_RECEIVED_FRAME_BROADCAST 0x34
#define WC_RECEIVED_MULTIFRAME 0x36
#define WC_REQ_SEND_SERVICE 0x80
#define WC_RES_SEND_SERVICE 0x81
#define WC_SERVICE_RESPONSE 0x82


#define WC_RADIO_PARAM_AWAKENING_PERIOD    0x00
#define WC_RADIO_PARAM_WAKE_UP_TYPE        0x01
#define WC_RADIO_PARAM_WAKE_UP_LENGTH      0x02
#define WC_RADIO_PARAM_RADIO_ACKNOLEDGE    0x04
#define WC_RADIO_PARAM_RADIO_ADDRESS       0x05
#define WC_RADIO_PARAM_RELAY_ROUTE         0x07
#define WC_RADIO_PARAM_POLLING_ROUTE       0x08
#define WC_RADIO_PARAM_GROUP_NUMBER        0x09
#define WC_RADIO_PARAM_POLLING_TIME        0x0A
#define WC_RADIO_PARAM_RADIO_USER_TIMEOUT  0x0C
#define WC_RADIO_PARAM_RECEPT_ERROR_STATUS 0x0E
#define WC_RADIO_PARAM_SWITCH_MODE_STATUS  0x10


#define update_crc(_byte) {			\
  uint8_t i;					\
  crc ^= _byte;					\
  for(i = 0; i < 8; i++) {			\
    uint8_t carry = crc & 0x1;			\
    crc /= 2;					\
    if (carry)					\
      crc ^= poly;				\
  }						\
}


#define WcPut1PayloadByte(_byte) { \
  WcPut1CtlByte(_byte); \
  update_crc(_byte);\
}

#define WcStart() \
  WcPut1CtlByte(WC_SYNC); \
  WcPut1CtlByte(WC_STX); \
  crc = 0;

#define WcEnd() \
  WcPut1CtlByte(crc&0xff); \
  WcPut1CtlByte(crc>>8); \
  WcPut1CtlByte(WC_ETX);

#define WcSendAck() \
  g_message("sending ACK"); \
  WcStart(); \
  WcPut1PayloadByte(WC_CTL_BYTE_LEN); \
  WcPut1PayloadByte(WC_ACK); \
  WcEnd()
  
#define WcSendFirmwareReq() \
  g_message("sending REQ_FIRMWARE_VERSION");	\
  WcStart();							\
  WcPut1PayloadByte(WC_CTL_BYTE_LEN); \
  WcPut1PayloadByte(WC_REQ_FIRMWARE_VERSION); \
  WcEnd()

#define WcSendReadRadioParamReq(no_param) \
  g_message("sending REQ_READ_RADIO_PARAM %d", no_param);	\
  WcStart(); \
  WcPut1PayloadByte( WC_CTL_BYTE_LEN + 1); \
  WcPut1PayloadByte(WC_REQ_READ_RADIO_PARAM); \
  WcPut1PayloadByte(no_param); \
  WcEnd()

#define WcSendWriteRadioParamReq(no_param, value)			\
  g_message("sending REQ_WRITE_RADIO_PARAM %d %d", no_param, value);		\
  WcStart(); \
  WcPut1PayloadByte( WC_CTL_BYTE_LEN + 2); \
  WcPut1PayloadByte(WC_REQ_WRITE_RADIO_PARAM); \
  WcPut1PayloadByte(no_param); \
  WcPut1PayloadByte(value); \
  WcEnd()

#define WcSendReqSendService(addr, type)						\
  {                                                                     \
    uint8_t i;								\
    g_message("sending REQ_SEND_SERVICE %02x %02x %02x %02x %02x %02x", addr[0], addr[1], addr[2] , addr[3], addr[4], addr[5]); \
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + WC_ADDR_LEN + 1);		\
    WcPut1PayloadByte(WC_REQ_SEND_SERVICE);				\
    WcPut1PayloadByte(addr[0]);\
    WcPut1PayloadByte(addr[1]);\
    WcPut1PayloadByte(addr[2]);\
    WcPut1PayloadByte(addr[3]);\
    WcPut1PayloadByte(addr[4]);\
    WcPut1PayloadByte(addr[5]);\
    WcPut1PayloadByte(type);						\
    WcEnd()								\
  }

#define WcSendMsg(addr, len, msg)					\
  {									\
    uint8_t i;								\
    GString* str = g_string_new( "sending REQ_SEND_MESSAGE " );		\
    wc_glib_append_addr(str, addr);					\
    g_string_append_printf(str, "%*s ", len, msg);			\
    g_message(str->str);						\
    g_string_free(str, TRUE);						\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + WC_ADDR_LEN + len );		\
    WcPut1PayloadByte(WC_REQ_SEND_MESSAGE);				\
    WcPut1PayloadByte(addr[0]);						\
    WcPut1PayloadByte(addr[1]);						\
    WcPut1PayloadByte(addr[2]);						\
    WcPut1PayloadByte(addr[3]);						\
    WcPut1PayloadByte(addr[4]);						\
    WcPut1PayloadByte(addr[5]);						\
    WcPut1PayloadByte(msg[0]);						\
    WcPut1PayloadByte(msg[1]);						\
    WcPut1PayloadByte(msg[2]);						\
    WcPut1PayloadByte(msg[3]);						\
    WcPut1PayloadByte(msg[4]);						\
    WcPut1PayloadByte(msg[5]);						\
    WcPut1PayloadByte(msg[6]);						\
    WcPut1PayloadByte(msg[7]);						\
    WcPut1PayloadByte(msg[8]);						\
    WcPut1PayloadByte(msg[9]);						\
    WcPut1PayloadByte(msg[10]);						\
    WcPut1PayloadByte(msg[11]);						\
    WcPut1PayloadByte(msg[12]);						\
    WcEnd()								\
      }									\
    
#define WcSendReqReadRemoteRssi(addr)					\
  {									\
    GString* str = g_string_new( "sending REQ_READ_REMOTE_RSSI " );		\
    wc_glib_append_addr(str, addr);					\
    g_message(str->str);						\
    g_string_free(str, TRUE);						\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + WC_ADDR_LEN );		\
    WcPut1PayloadByte(WC_REQ_READ_REMOTE_RSSI);				\
    WcPut1PayloadByte(addr[0]);						\
    WcPut1PayloadByte(addr[1]);						\
    WcPut1PayloadByte(addr[2]);						\
    WcPut1PayloadByte(addr[3]);						\
    WcPut1PayloadByte(addr[4]);						\
    WcPut1PayloadByte(addr[5]);						\
    WcEnd()								\
      }									\

#define WcSendReqReadLocalRssi(addr)					\
  {									\
    GString* str = g_string_new( "sending REQ_READ_LOCAL_RSSI " );	\
    wc_glib_append_addr(str, addr);					\
    g_message(str->str);						\
    g_string_free(str, TRUE);						\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + WC_ADDR_LEN );			\
    WcPut1PayloadByte(WC_REQ_READ_LOCAL_RSSI);				\
    WcPut1PayloadByte(addr[0]);						\
    WcPut1PayloadByte(addr[1]);						\
    WcPut1PayloadByte(addr[2]);						\
    WcPut1PayloadByte(addr[3]);						\
    WcPut1PayloadByte(addr[4]);						\
    WcPut1PayloadByte(addr[5]);						\
    WcEnd()								\
      }									\

#define WcSendReqReadPhyconfig()					\
  {									\
    g_message("sending REQ_READ_PHYCONFIG ");				\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN );				\
    WcPut1PayloadByte(WC_REQ_READ_PHYCONFIG);				\
    WcEnd()								\
  }									\

#define WcSendReqSelectPhyconfig(cfg)				\
  {									\
    g_message("sending REQ_SELECT_PHYCONFIG ");				\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + 2);				\
    WcPut1PayloadByte(WC_REQ_SELECT_PHYCONFIG);				\
    WcPut1PayloadByte(cfg>>8);				                \
    WcPut1PayloadByte(cfg&0xff);				                \
    WcEnd()								\
  }									\
    
#define WcSendReqReadChannel()					\
  {									\
    g_message("sending REQ_READ_CHANNEL ");				\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN );				\
    WcPut1PayloadByte(WC_REQ_READ_CHANNEL);				\
    WcEnd()								\
  }									\

#define WcSendReqSelectChannel(channel)					\
  {									\
    g_message("sending REQ_SELECT_CHANNEL ");				\
    WcStart();								\
    WcPut1PayloadByte(WC_CTL_BYTE_LEN + 1);				\
    WcPut1PayloadByte(WC_REQ_SELECT_CHANNEL);				\
    WcPut1PayloadByte(channel);				                \
    WcEnd()								\
  }									\
    
    
#endif /* WAVECARD_H */
