(*
 * $Id$
 *
 * Copyright (C) 2006 ENAC, Pascal Brisset, Antoine Drouin
 * Copyright (C) 2007 UofA, Roman Krashanitsa
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
  let start_delimiter = Char.chr 0x81
  let offset_length = 1
  let offset_payload = 0
  let size_packet = 4
  let index_start = fun s ->
		String.index s start_delimiter	

  let length = fun s i ->
(*    for j=i to String.length s -1 do Printf.fprintf stdout "%x " (int_of_char s.[j]) done;
    Printf.fprintf stdout "\n";flush stdout; *)
    if String.length s < 2 then raise Serial.Not_enough; 
    match int_of_char s.[0] with
      0x81 -> int_of_char s.[1] + 7
    | 0x82 -> 4
    | 0x83 -> int_of_char s.[1] + 5
    | _ -> raise Serial.Not_enough

  let compute_checksum = fun s ->
    let cs = ref 0 in
    for i = offset_payload to String.length s - 2 do
      cs := (!cs + Char.code s.[i]) land 0xff
    done;
    0xff - !cs

  let checksum = fun s ->
	true

  let payload = fun s ->
    Serial.payload_of_string (String.sub s offset_payload (String.length s))

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

type frame =
    API_Transmit_Packet of addr16 * int * string
  | API_Send_Data_Complete of int * int
  | API_Receive of addr16 * string
  | API_Enchanced_Receive of addr16 * int * string


let api_tx_id = Char.chr 0x81
let api_tx_status_id = Char.chr 0x82
let api_rx_id = Char.chr 0x83
let api_rx_enchanced_id = Char.chr 0x81
let api_at_command_response_id = Char.chr 0x88
let retries = 4

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

  
let api_tx = fun dest retries data ->
  let n = String.length data in
  assert (n <= 100);
  Printf.fprintf stdout "sending to %x\n" dest; flush stdout;
  let l = 1 + 1 + 1 + 1 + 3 + n in
  let s = String.create l in
  s.[0] <- api_tx_id;
  s.[1] <- Char.chr n;
  s.[2] <- Char.chr 0;
  s.[3] <- Char.chr retries;
  s.[4] <- Char.chr 0;
  assert (dest >= 0 && dest < 0x10000);
  write_int16 s 5 dest;

  String.blit data 0 s 7 n;
  s
  

let at_command_sequence = "AT+++\x0D"
let at_set_my = fun addr ->
  assert (addr >= 0 && addr < 0x10000);
  Printf.sprintf "\xCC\xC1\x80\x06\x00\x50\x67\x00%c%c" (Char.chr (addr lsr 8)) (Char.chr (addr land 0xff)) 
  

let baud_rates = [1200, 0x40; 2400, 0xA0; 4800, 0xD0; 9600, 0xE8; 19200, 0xF4; 
		  38400, 0xFA; 57600, 0xFC; 115200,  0xFE]
let at_set_baud_rate = fun baud ->
  try
    Printf.sprintf "\xCC\xC1\x42\x01%c" (Char.chr (List.assoc baud baud_rates))
  with
    Not_found -> invalid_arg "at_set_baud_rate"

let at_exit = "\xCC\x41\x54\x4F\r"
let at_api_enable = "\xCC\x17\x0B"


let api_tx_id = Char.chr 0x81
let api_tx_status_id = Char.chr 0x82
let api_rx_id = Char.chr 0x83
let api_rx_enchanced_id = Char.chr 0x81


let api_parse_frame = fun s ->
  let n = String.length s in
  assert(n>0);
(*  Printf.fprintf stdout "%x\n" (int_of_char s.[0]); flush stdout; *)
  match s.[0] with
    x when x = api_tx_status_id ->
      assert(n = 4);
      API_Send_Data_Complete (Char.code s.[2], Char.code s.[3])
  | x when x = api_rx_id ->
     API_Receive (read_int16 s 3, String.sub s 5 (n-5))
(*API_Enchanced_Receive (0x000B,  0x0B, "\x01\x00\x00")*)
  | x when x = api_rx_enchanced_id ->
      (* tx here allows to receive simulated aerocomm messages *)
	(*Printf.fprintf stdout "AC=%d\n" (read_int16 s 5); flush stdout; *) 
      API_Enchanced_Receive (read_int16 s 5,  Char.code s.[3], String.sub s 7 (n-7))
  | x -> failwith (Printf.sprintf "Aerocomm.parse_frame: unknown frame id '%d'" (Char.code x))

