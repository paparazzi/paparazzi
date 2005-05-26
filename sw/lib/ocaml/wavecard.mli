(*
 * $Id$
 *
 * Coronis wavecard handling
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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
 *)

type cmd_name =
    ACK
  |  NAK
  |  ERROR
  |  REQ_WRITE_RADIO_PARAM
  |  RES_WRITE_RADIO_PARAM
  |  REQ_READ_RADIO_PARAM
  |  RES_READ_RADIO_PARAM
  |  REQ_SELECT_CHANNEL
  |  RES_SELECT_CHANNEL
  |  REQ_READ_CHANNEL
  |  RES_READ_CHANNEL
  |  REQ_SELECT_PHYCONFIG
  |  RES_SELECT_PHYCONFIG
  |  REQ_READ_PHYCONFIG
  |  RES_READ_PHYCONFIG
  |  REQ_READ_REMOTE_RSSI
  |  RES_READ_REMOTE_RSSI
  |  REQ_READ_LOCAL_RSSI
  |  RES_READ_LOCAL_RSSI
  |  REQ_FIRMWARE_VERSION
  |  RES_FIRMWARE_VERSION
  |  MODE_TEST
  |  REQ_SEND_FRAME
  |  RES_SEND_FRAME
  |  REQ_SEND_MESSAGE
  |  REQ_SEND_POLLING
  |  REQ_SEND_BROADCAST
  |  RECEIVED_FRAME
  |  RECEPTION_ERROR
  |  RECEIVED_FRAME_POLLING
  |  RECEIVED_FRAME_BROADCAST
  |  RECEIVED_MULTIFRAME
  |  REQ_SEND_SERVICE
  |  RES_SEND_SERVICE
  |  SERVICE_RESPONSE

type data = string
type cmd = cmd_name * data

val send :  Unix.file_descr -> cmd -> unit

val receive : ?ack:(unit -> unit) -> (cmd -> 'a) -> (Unix.file_descr -> unit)

val code_of_cmd : cmd_name -> int
