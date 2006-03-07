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
(** available commands *)


val code_of_cmd : cmd_name -> int
(** Code of command *)

type data = string
type cmd = cmd_name * data
(** A command is composed of a command name and some untyped data *)

type addr
val addr_of_string : string -> addr
(** [addr_of_string address] where [address] is a 64 bits number, for example
[0x011804c0012d] *)

val send :  Unix.file_descr -> cmd -> unit
(** Send a command on the channel connected to the serial port of the wavecard *)

val send_addressed : Unix.file_descr -> (cmd_name*addr*data) -> unit
(** [send_addressed fd (cmd, a, data)] Sends [cmd] with data obtained by
concatenation of codinf of [a] and [data] *)

val receive : ?ack:(unit -> unit) -> (cmd -> 'a) -> (Unix.file_descr -> unit)
(** [receive ?acknowledger callbkack] Returns a listener for wavecard messages *)


val compute_checksum : string -> int
(** [compute_checksum buf] Computes the checksum of a complete message buffer,
including the header of the message *)
