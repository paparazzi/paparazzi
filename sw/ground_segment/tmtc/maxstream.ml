(*
 * $Id$
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

open Latlong
open Printf
module W = Wavecard
module Tm_Pprz = Pprz.Protocol(struct let name = "telemetry_fbw" end)
module Ground_Pprz = Pprz.Protocol(struct let name = "ground" end)
module Dl_Pprz = Pprz.Protocol(struct let name = "datalink" end)
module PprzTransport = Serial.Transport(Tm_Pprz)

let ground_id = 0


type modem = { fd : Unix.file_descr; }

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

let use_tele_message = fun buf ->
  Debug.call 'm' (fun f -> fprintf f "mm receiving: "; for i = 0 to String.length buf - 1 do fprintf f "%x " (Char.code buf.[i]) done; fprintf f "\n");
  let (msg_id, ac_id, values) = Tm_Pprz.values_of_bin buf in
  let msg = Tm_Pprz.message_of_id msg_id in
  Tm_Pprz.message_send (string_of_int ac_id) msg.Pprz.name values

let buffer = ref ""

let maxstream_parse = fun buf ->
  let b = !buffer ^ buf in 
  let nb_used = PprzTransport.parse use_tele_message b in
  buffer := String.sub b nb_used (String.length b - nb_used);
  String.length buf

let maxstream_receive = Serial.input (fun b -> maxstream_parse b)



let send = fun ac s ->
(*  Wavecard.send_addressed ac.fd (W.REQ_SEND_MESSAGE,ac.addr,s) *)
  maxstream_send ac.fd s

let send_dl_msg = fun ac a ->
  let (id, values) = Dl_Pprz.values_of_string a in
  let s  = Dl_Pprz.payload_of_values id ground_id values in
  send ac s
    

let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f))

(** Got a FLIGHT_PARAM message and dispatch a ACINFO *)
let get_fp = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
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
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s

(** Got a MOVE_WAYPOINT and send a MOVE_WP *)
let move_wp = fun ac _sender vs ->
  Debug.call 'm' (fun f -> fprintf f "mm MOVE WAYPOINT\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  let f = fun a -> Pprz.float_assoc a vs in
  let lat = f "lat"
  and long = f "long"
  and alt = f "alt"
  and wp_id = Pprz.int_assoc "wp_id" vs in
  let wgs84 = {posn_lat=(Deg>>Rad)lat;posn_long=(Deg>>Rad)long} in
  let utm = Latlong.utm_of WGS84 wgs84 in
  let vs = ["wp_id", Pprz.Int wp_id;
	    "utm_east", cm_of_m utm.utm_x;
	    "utm_north", cm_of_m utm.utm_y;
	    "alt", cm_of_m alt] in
  let msg_id, _ = Dl_Pprz.message_of_name "MOVE_WP" in
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s

(** Got a SEND_EVENT, and send an EVENT *)
let send_event = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  let ev_id = Pprz.int_assoc "event_id" vs in
  let vs = ["event", Pprz.Int ev_id] in
  let msg_id, _ = Dl_Pprz.message_of_name "EVENT" in
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s
    

(** Got a DL_SETTING, and send an SETTING *)
let setting = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  let idx = Pprz.int_assoc "index" vs in
  let vs = ["event", Pprz.Int idx; "value", List.assoc "value" vs] in
  let msg_id, _ = Dl_Pprz.message_of_name "SETTING" in
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s

(** Got a JUMP_TO_BLOCK, and send an BLOCK *)
let jump_block = fun ac _sender vs ->
  Debug.call 'm' (fun f -> fprintf f "mm JUMP_TO_BLOCK\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  let block_id = Pprz.int_assoc "block_id" vs in
  let vs = ["block_id", Pprz.Int block_id] in
  let msg_id, _ = Dl_Pprz.message_of_name "BLOCK" in
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s
    

let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in

  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "Ivy bus (%s)" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "Port (%s)" !port)] in
  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  Ivy.init "Maxstream connect" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;
  
  try
    let fd = Serial.opendev !port Serial.B38400 in
    let ac = { fd=fd; } in
    (* Listening on wavecard *)
    let cb = fun _ ->
      maxstream_receive fd;
      true in

    ignore (Glib.Io.add_watch [`IN] cb (GMain.Io.channel_of_descr fd));

    (* Sending request from Ivy *)


    (** Listening on Ivy *)
(**    ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" (get_fp ac)); *)
    ignore (Ground_Pprz.message_bind "MOVE_WAYPOINT" (move_wp ac));
    ignore (Ground_Pprz.message_bind "SEND_EVENT" (send_event ac));
    ignore (Ground_Pprz.message_bind "DL_SETTING" (setting ac));
    ignore (Ground_Pprz.message_bind "JUMP_TO_BLOCK" (jump_block ac));
    (* For debug *)
    ignore (Ivy.bind (fun _ a -> send_dl_msg ac a.(0)) "TO_WAVECARD +(.*)");


    (* Main Loop *)
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
  with
    exn -> fprintf stderr "%s\n" (Printexc.to_string exn)
