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

open Printf

type cmd_name = string
type data = string

type cmd = cmd_name * string

let cmd_names = [
  0x06, "ACK";
  0x15, "NAK";
  0x00, "ERROR";
  0x40, "REQ_WRITE_RADIO_PARAM";
  0x41, "RES_WRITE_RADIO_PARAM";
  0x50, "REQ_READ_RADIO_PARAM";
  0x51, "RES_READ_RADIO_PARAM";
  0x60, "REQ_SELECT_CHANNEL";
  0x61, "RES_SELECT_CHANNEL";
  0x62, "REQ_READ_CHANNEL";
  0x63, "RES_READ_CHANNEL";
  0x64, "REQ_SELECT_PHYCONFIG";
  0x65, "RES_SELECT_PHYCONFIG";
  0x66, "REQ_READ_PHYCONFIG";
  0x67, "RES_READ_PHYCONFIG";
  0x68, "REQ_READ_REMOTE_RSSI";
  0x69, "RES_READ_REMOTE_RSSI";
  0x6A, "REQ_READ_LOCAL_RSSI";
  0x6B, "RES_READ_LOCAL_RSSI";
  0xA0, "REQ_FIRMWARE_VERSION";
  0xA1, "RES_FIRMWARE_VERSION";
  0xB0, "MODE_TEST";
  0x20, "REQ_SEND_FRAME";
  0x21, "RES_SEND_FRAME";
  0x22, "REQ_SEND_MESSAGE";
  0x26, "REQ_SEND_POLLING";
  0x28, "REQ_SEND_BROADCAST";
  0x30, "RECEIVED_FRAME";
  0x31, "RECEPTION_ERROR";
  0x32, "RECEIVED_FRAME_POLLING";
  0x34, "RECEIVED_FRAME_BROADCAST";
  0x36, "RECEIVED_MULTIFRAME";
  0x80, "REQ_SEND_SERVICE";
  0x81, "RES_SEND_SERVICE";
  0x82, "SERVICE_RESPONSE"]

let rec cossa = fun x l ->
  match l with
    [] -> raise Not_found
  | (v, k)::xs -> if k = x then v else cossa x xs

let cmd_name_of = fun x -> try List.assoc x cmd_names with Not_found -> failwith (sprintf "Unknown command: %2x" x)
let of_cmd_name = fun x -> try cossa x cmd_names with Not_found -> failwith (sprintf "Unknown command: %s" x)

let sync = Char.chr 0xff
let stx = Char.chr 0x02
let etx = Char.chr 0x03

let length = fun buf -> Char.code buf.[2]
let total_length = fun buf -> length buf + 3

let payload = fun buf ->
  let l = length buf in
  let data = String.sub buf 4 (l-4) in
  (cmd_name_of (Char.code buf.[3]), data)

let (^=) r x = r := !r lxor x

let compute_checksum =
  let poly = 0x8408 in
  fun buf ->
    let lg = length buf - 2
    and crc = ref 0 in
    for j = 0 to lg - 1 do
      crc ^= Char.code buf.[j+2];
      for i = 0 to 7 do
	let carry = !crc land 0x01 = 1 in
	crc := !crc lsr 1;
	if carry then
	  crc ^= poly
      done
    done;
    !crc

let checksum = fun buf ->
  let crc = compute_checksum buf
  and l = length buf in
  crc land 0xff = Char.code buf.[l] && crc lsr 8 = Char.code buf.[l+1]
    
  


let parse = fun buf ?ack f ->
  let n = String.length buf in
  if n < 3 || n < total_length buf then
    0 (* Not enough chars to read *)
  else if buf.[0] <> sync then
    1
  else if buf.[1] <> stx || not (checksum buf) then
    2
  else begin
    f (payload buf);
    begin
      match ack with
	None -> ()
      | Some ack -> ack ()
    end;
    total_length buf
  end
    

let receive = fun ?ack f ->
  Serial.input (fun b -> parse b ?ack f)

let send = fun fd (cmd, data) ->
  let l = String.length data + 4 in
  if l >= 256 then
    invalid_arg "Wavecard.send";
  let buf = String.create (l+3) in
  buf.[0] <- sync;
  buf.[1] <- stx;
  buf.[2] <- Char.chr l;
  buf.[3] <- Char.chr (of_cmd_name cmd);
  for i = 4 to l - 1 do
    buf.[i] <- data.[i-4]
  done;
  let crc = compute_checksum buf in
  buf.[l] <- Char.chr (crc land 0xff);
  buf.[l+1] <- Char.chr (crc lsr 8);
  buf.[l+2] <- etx;
  let o = Unix.out_channel_of_descr fd in
  Printf.fprintf o "%s" buf;
  flush o
