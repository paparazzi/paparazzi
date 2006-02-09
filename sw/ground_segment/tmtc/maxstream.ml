(*
 * $Id$
 *
 * Multi aircrafts receiver, logger and broadcaster
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
module W = Wavecard
module Tm_Pprz = Pprz.Protocol(struct let name = "telemetry_ap" end)
module Ground_Pprz = Pprz.Protocol(struct let name = "ground" end)
module Dl_Pprz = Pprz.Protocol(struct let name = "datalink" end)

let ground_id = 0

type aircraft = { fd : Unix.file_descr; id : int; addr : W.addr }

(*let send_ack = fun delay fd () ->
  ignore (GMain.Timeout.add delay (fun _ -> W.send fd (W.ACK, ""); false)) *)

let sync = Char.chr 0xff
let stx = Char.chr 0x02
let etx = Char.chr 0x03
let wc_received_frame =  Char.chr 0x30

let maxstream_send = fun fd data ->
  let l = String.length data + 4 + 6 in
  let buf = String.create (l+3) in
  buf.[0] <- sync;
  buf.[1] <- stx;
  buf.[2] <- Char.chr l;
  buf.[3] <- wc_received_frame;
  buf.[4] <- Char.chr 0;
  buf.[5] <- Char.chr 0;
  buf.[6] <- Char.chr 0;
  buf.[7] <- Char.chr 0;
  buf.[8] <- Char.chr 0;
  buf.[9] <- Char.chr 0;
 for i = 10 to l - 1 do
    buf.[i] <- data.[i-10]
  done;
  let crc = Wavecard.compute_checksum buf in
  buf.[l] <- Char.chr (crc land 0xff);
  buf.[l+1] <- Char.chr (crc lsr 8);
  buf.[l+2] <- etx;
  let o = Unix.out_channel_of_descr fd in
  Printf.fprintf o "%s" buf;
  Debug.call 'm' (fun f -> fprintf f "mm sending: "; for i = 0 to String.length buf - 1 do fprintf f "%x " (Char.code buf.[i]) done; fprintf f "\n");
  flush o

let maxstream_parse = fun buf f ->
  10

let maxstream_receive = fun f ->
  Serial.input (fun b -> maxstream_parse b f)


let send = fun ac s ->
(*  Wavecard.send_addressed ac.fd (W.REQ_SEND_MESSAGE,ac.addr,s) *)
  maxstream_send ac.fd s

let send_dl_msg = fun ac a ->
  let (id, values) = Dl_Pprz.values_of_string a in
  let s  = Dl_Pprz.payload_of_values id ground_id values in
  send ac s
    
let broadcast_msg = fun ac (com, data) ->
  match com with
    W.RECEIVED_FRAME ->
      Ivy.send (sprintf "FROM_WAVECARD %s" data)
  | W.RES_READ_REMOTE_RSSI ->
      Tm_Pprz.message_send (string_of_int ac.id) "WC_RSSI" ["raw_level", Pprz.Int (Char.code data.[0])];
      Debug.call 'm' (fun f -> fprintf f "mm remote RSSI %d\n" (Char.code data.[0]))
  | W.RES_READ_RADIO_PARAM ->
      Ivy.send (sprintf "WC_ADDR %s" data);
      Debug.call 'm' (fun f -> fprintf f "mm local addr : " ; for i = 0 to String.length data - 1 do fprintf f "%x " (Char.code data.[i]) done; fprintf f "\n")
  | _ -> 
      Debug.call 'm' (fun f -> fprintf f "mm receiving: %02x " (W.code_of_cmd com); for i = 0 to String.length data - 1 do fprintf f "%x " (Char.code data.[i]) done; fprintf f "\n");
      Ivy.send (sprintf "RAW_FROM_WAVECARD %2x %s" (W.code_of_cmd com) data)


let rssi_period = 5000 (** ms *)
let req_rssi = fun ac ->
(*  Wavecard.send_addressed ac.fd (W.REQ_READ_REMOTE_RSSI, ac.addr,"") *)
  Debug.call 'm' (fun f -> fprintf f "mm PERIODIC\n")
 


let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f))

(** Got a FLIGHT_PARAM message and dispatch a ACINFO *)
let get_fp = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  if ac_id <> ac.id then
    let f = fun a -> Pprz.float_assoc a vs in
    let ux = f "east"
    and uy = f "north"
    and course = f "course"
    and alt = f "alt"
    and gspeed = f "speed" in
    let vs = ["ac_id", Pprz.Int ac_id;
	      "utm_east", cm_of_m ux;
	      "utm_north", cm_of_m uy;
	      "course", Pprz.Int (truncate (10. *. course));
	      "alt", cm_of_m alt;
	      "speed", cm_of_m gspeed] in
    let msg_id, _ = Dl_Pprz.message_of_name "ACINFO" in
    let s = Dl_Pprz.payload_of_values msg_id ground_id vs in
    send ac s

(** Got a MOVE_WAYPOINT and send a MOVE_WP *)
let move_wp = fun ac _sender vs ->
  Debug.call 'm' (fun f -> fprintf f "mm MOVE WAYPOINT\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  if ac_id = ac.id then
    let f = fun a -> Pprz.float_assoc a vs in
    let ux = f "utm_east"
    and uy = f "utm_north"
    and alt = f "alt"
    and wp_id = Pprz.int_assoc "wp_id" vs in
    let vs = ["wp_id", Pprz.Int wp_id;
	      "utm_east", cm_of_m ux;
	      "utm_north", cm_of_m uy;
	      "alt", cm_of_m alt] in
    let msg_id, _ = Dl_Pprz.message_of_name "MOVE_WP" in
    let s = Dl_Pprz.payload_of_values msg_id ground_id vs in
    send ac s

(** Got a SEND_EVENT, and send an EVENT *)
let send_event = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  if ac_id = ac.id then
    let ev_id = Pprz.int_assoc "event_id" vs in
    let vs = ["event", Pprz.Int ev_id] in
    let msg_id, _ = Dl_Pprz.message_of_name "EVENT" in
    let s = Dl_Pprz.payload_of_values msg_id ground_id vs in
    send ac s
    

(** Got a DL_SETTING, and send an SETTING *)
let setting = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  if ac_id = ac.id then
    let idx = Pprz.int_assoc "index" vs in
    let vs = ["event", Pprz.Int idx; "value", List.assoc "value" vs] in
    let msg_id, _ = Dl_Pprz.message_of_name "SETTING" in
    let s = Dl_Pprz.payload_of_values msg_id ground_id vs in
    send ac s

(** Got a JUMP_TO_BLOCK, and send an BLOCK *)
let jump_block = fun ac _sender vs ->
  Debug.call 'm' (fun f -> fprintf f "mm JUMP_TO_BLOCK\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  if ac_id = ac.id then
    let block_id = Pprz.int_assoc "block_id" vs in
    let vs = ["block_id", Pprz.Int block_id] in
    let msg_id, _ = Dl_Pprz.message_of_name "BLOCK" in
    let s = Dl_Pprz.payload_of_values msg_id ground_id vs in
    send ac s


let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in
  let distant_addr = ref "0x011804c0012d" in
  let id = ref 4 in

  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "Ivy bus (%s)" !ivy_bus);
      "-id", Arg.Set_int id, (sprintf "A/C Id (%d)" !id);
      "-a", Arg.Set_string distant_addr, (sprintf "Distant address (%s)" !distant_addr);
      "-d", Arg.Set_string port, (sprintf "Port (%s)" !port)] in
  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  Ivy.init "Maxstream connect" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;
  
  try
    let fd = Serial.opendev !port Serial.B9600 in
    let ac = { fd=fd; id= !id; addr= W.addr_of_string !distant_addr } in
    (* Listening on wavecard *)
(*    let listener = Wavecard.receive ~ack:(send_ack 10 fd) (broadcast_msg ac) in *)
    let listener = maxstream_receive (broadcast_msg ac) in
    let cb = fun _ ->
      listener fd;
      true in

    ignore (Glib.Io.add_watch [`IN] cb (GMain.Io.channel_of_descr fd));

    (* Sending request from Ivy *)


    (** Listening on Ivy *)
(*    ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" (get_fp ac)); *)
    ignore (Ground_Pprz.message_bind "MOVE_WAYPOINT" (move_wp ac));
    ignore (Ground_Pprz.message_bind "SEND_EVENT" (send_event ac));
    ignore (Ground_Pprz.message_bind "DL_SETTING" (setting ac));
    ignore (Ground_Pprz.message_bind "JUMP_TO_BLOCK" (jump_block ac));
    (* For debug *)
    ignore (Ivy.bind (fun _ a -> send_dl_msg ac a.(0)) "TO_WAVECARD +(.*)");


    ignore (GMain.Timeout.add rssi_period (fun _ -> req_rssi ac; true));
    (* request own address *)
(*    let s = String.make 1 (char_of_int 5) in
    Wavecard.send ac.fd (W.REQ_READ_RADIO_PARAM, s);
*)

    (* Main Loop *)
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
  with
    exn -> fprintf stderr "%s\n" (Printexc.to_string exn)
      



