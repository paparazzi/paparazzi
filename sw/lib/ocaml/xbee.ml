(*
 * $Id$
 *
 * Copyright (C) 2006 ENAC, Pascal Brisset, Antoine Drouin
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


module Protocol = struct
  let start_delimiter = Char.chr 0x7e
  let offset_length = 1
  let offset_payload = 3
  let size_packet = 4
  let index_start = fun s ->
    String.index s start_delimiter

  let length = fun s i ->
    if String.length s < i+offset_length+2 then
      raise Serial.Not_enough
    else
      (Char.code s.[i+offset_length] lsl 8) lor (Char.code s.[i+offset_length+1]) + size_packet

  let compute_checksum = fun s ->
    let cs = ref 0 in
    for i = offset_payload to String.length s - 2 do
      cs := (!cs + Char.code s.[i]) land 0xff
    done;
    0xff - !cs

  let checksum = fun s ->
    let c = compute_checksum s in
    Debug.call 'x' (fun f -> Printf.fprintf f "BX.cs=%x\n" c);
    c = Char.code s.[String.length s-1]

  let payload = fun s ->
    Serial.payload_of_string (String.sub s offset_payload (String.length s - size_packet))

  let packet = fun payload ->
    let payload = Serial.string_of_payload payload in
    let n = String.length payload in
    let msg_length = n + size_packet in
    let m = String.create msg_length in
    String.blit payload 0 m offset_payload n;
    m.[0] <- start_delimiter;
    m.[offset_length] <- Char.chr (n lsr 8);
    m.[offset_length+1] <- Char.chr (n land 0xff);
    let cs = compute_checksum m in
    m.[msg_length-1] <- Char.chr cs;
    m
end

type frame_data = string
type frame_id = int
type addr64 = Int64.t
type addr16 = int
type byte = int
type rssi = int
type frame =
    Modem_Status of int
  | AT_Command_Response of frame_id * string * int * string
  | TX_Status of frame_id * byte
  | TX868_Status of frame_id * byte * int
  | RX_Packet_64 of addr64 * int * int * string
  | RX868_Packet of addr64 * byte * string
  | RX_Packet_16 of addr16 * int * int * string


let mode868 = ref false

let check_not_in_868 = fun s ->
  if !mode868 then
    failwith (Printf.sprintf "Xbee.%s not available in mode868" s)


let api_tx64_id = Char.chr 0x00
let api868_tx64_id = Char.chr 0x10
let api_tx16_id = Char.chr 0x01
let api_rx64_id = Char.chr 0x80
let api_rx16_id = Char.chr 0x81
let api_at_command_response_id = Char.chr 0x88
let api_tx_status_id = Char.chr 0x89
let api868_tx_status_id = Char.chr 0x8b
let api_modem_status_id = Char.chr 0x8a
let api_rx64_id = Char.chr 0x80
let api868_rx64_id = Char.chr 0x90
let api_rx16_id = Char.chr 0x81

let write_int64 = fun buf offset x ->
  for i = 0 to 7 do
    buf.[offset+7-i] <- Char.chr (Int64.to_int (Int64.shift_right x (8*i)) land 0xff)
  done

let read_int64 = fun buf offset ->
  let x = ref Int64.zero in
  for i = 0 to 7 do
    x := Int64.logor (Int64.shift_left !x 8) (Int64.of_int (Char.code buf.[offset+i]))
  done;
  !x

let write_int16 = fun buf offset x ->
  buf.[offset] <- Char.chr (x lsr 8);
  buf.[offset+1] <- Char.chr (x land 0xff)

let read_int16 = fun buf offset ->
  (Char.code buf.[offset] lsl 8) lor (Char.code buf.[offset+1])

let api_tx64 = fun ?(frame_id = 0) dest data ->
  assert (frame_id >=0 && frame_id < 0x100);
  let n = String.length data in
  assert (n <= 100);
  let optional868 = if !mode868 then 3 else 0 in
  let l = 1 + 1 + 8 + optional868 + 1 + n in
  let s = String.create l in
  s.[0] <- api_tx64_id;
  s.[1] <- Char.chr frame_id;
  if !mode868 then begin
    s.[10] <- Char.chr 0xff;
    s.[11] <- Char.chr 0xfe;
    s.[12] <- Char.chr 0x0;
  end;
  write_int64 s 2 dest;
  s.[10+optional868] <- Char.chr 0;
  String.blit data 0 s (11+ optional868) n;
  s

let api_tx16 = fun ?(frame_id = 0) dest data ->
  check_not_in_868 "api_tx16";
  assert (frame_id >=0 && frame_id < 0x100);
  let n = String.length data in
  assert (n <= 100);
  let l = 1 + 1 + 2 + 1 + n in
  let s = String.create l in
  s.[0] <- api_tx16_id;
  s.[1] <- Char.chr frame_id;
  assert (dest >= 0 && dest < 0x10000);
  write_int16 s 2 dest;
  s.[4] <- Char.chr 0;
  String.blit data 0 s 5 n;
  s


let at_command_sequence = "+++"
let at_set_my = fun addr ->
  assert (addr >= 0 && addr < 0x10000);
  Printf.sprintf "ATMY%04x\r" addr
let baud_rates = [1200, 0; 2400, 1; 4800, 2; 9600, 3; 19200, 4;
		  38400, 5; 57600, 6; 115200,  7]
let at_set_baud_rate = fun baud ->
  try
    Printf.sprintf "ATBD%d\r" (List.assoc baud baud_rates)
  with
    Not_found -> invalid_arg "at_set_baud_rate"

let at_exit = "ATCN\r"
let at_api_enable = "ATAP1\r"

let api_parse_frame = fun s ->
  let n = String.length s in
  assert(n>0);
  match s.[0] with
    x when x = api_at_command_response_id ->
      assert(n >= 5);
      AT_Command_Response (Char.code s.[1], String.sub s 2 2,
			   Char.code s.[4], String.sub s 5 (n-5))
  | x when not !mode868 && x = api_tx_status_id ->
      assert(n = 3);
      TX_Status (Char.code s.[1], Char.code s.[2])
  | x when !mode868 && x = api868_tx_status_id ->
      assert(n = 7);
      TX868_Status (Char.code s.[1], Char.code s.[5], Char.code s.[4])
  | x when x = api_modem_status_id ->
      Modem_Status (Char.code s.[1])
  | x when not !mode868 && x = api_rx64_id ->
      assert(n >= 11);
      RX_Packet_64 (read_int64 s 1, Char.code s.[9],
		    Char.code s.[10], String.sub s 11 (n-11))
  | x when !mode868 && x = api868_rx64_id ->
      let idx_data = 12 in
      assert(n >= idx_data);
      RX868_Packet (read_int64 s 1,
		    Char.code s.[11], String.sub s idx_data (n-idx_data))
  | x when not !mode868 && (x = api_rx16_id || x = api_tx16_id) ->
      (* tx16 here allows to receive simulated xbee messages *)
      RX_Packet_16 (read_int16 s 1, Char.code s.[3], Char.code  s.[4], String.sub s 5 (n-5))
  | x -> failwith (Printf.sprintf "Xbee.parse_frame: unknown frame id '%d'" (Char.code x))

