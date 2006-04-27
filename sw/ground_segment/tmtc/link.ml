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
module Tm_Pprz = Pprz.Messages(struct let name = "telemetry" end)
module Ground_Pprz = Pprz.Messages(struct let name = "ground" end)
module Dl_Pprz = Pprz.Messages(struct let name = "datalink" end)
module PprzTransport = Serial.Transport(Pprz.Transport)

let ground_id = 0

type transport =
    Modem
  | Pprz
  | Wavecard
  | Maxstream

let transport_of_string = function
    "modem" -> Modem
  | "pprz" -> Pprz
  | "wavecard" -> Wavecard
  | "maxstream" -> Maxstream
  | x -> invalid_arg (sprintf "transport_of_string: %s" x)

type modem = { fd : Unix.file_descr; transport : transport }

(*let send_ack = fun delay fd () ->
  ignore (GMain.Timeout.add delay (fun _ -> W.send fd (W.ACK, ""); false)) *)

let sync = Char.chr 0xff
let stx = Char.chr 0x02
let etx = Char.chr 0x03
let wc_received_frame =  Char.chr 0x30

let use_tele_message = fun payload ->
  Debug.call 'm' (fun f -> let buf = Serial.string_of_payload payload in fprintf f "mm receiving: "; for i = 0 to String.length buf - 1 do fprintf f "%x " (Char.code buf.[i]) done; fprintf f "\n");
  let (msg_id, ac_id, values) = Tm_Pprz.values_of_payload payload in
  let msg = Tm_Pprz.message_of_id msg_id in
  Tm_Pprz.message_send (string_of_int ac_id) msg.Pprz.name values

let send = fun ac payload ->
  match ac.transport with
    Pprz ->
      let o = Unix.out_channel_of_descr ac.fd in
      let buf = Pprz.Transport.packet payload in
      Printf.fprintf o "%s" buf; flush o;
      Debug.call 'm' (fun f -> fprintf f "mm sending: %s\n" (Debug.xprint buf));
  | _ -> failwith "send: not yet"


let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f))

(** Got a FLIGHT_PARAM message and dispatch a ACINFO *)
let get_fp = fun ac _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  let f = fun a -> Pprz.float_assoc a vs in
  let lat = (Deg>>Rad) (f "lat")
  and long = (Deg>>Rad) (f "long")
  and course = f "course"
  and alt = f "alt"
  and gspeed = f "speed" in
  let utm = Latlong.utm_of WGS84 {posn_lat=lat; posn_long=long} in
  let vs = ["ac_id", Pprz.Int ac_id;
	    "utm_east", cm_of_m utm.utm_x;
	    "utm_north", cm_of_m utm.utm_y;
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

(** Got a RAW_DATALINK message *)
let raw_datalink = fun ac _sender vs ->
  let m = Pprz.string_assoc "message" vs in
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  for i = 0 to String.length m - 1 do
    if m.[i] = ';' then m.[i] <- ' '
  done;
  let msg_id, vs = Dl_Pprz.values_of_string m in
  let s = Dl_Pprz.payload_of_values msg_id ac_id vs in
  send ac s

module PprzModem = struct
  type status = {
      mutable rx_byte : int;
      mutable rx_msg : int;
      mutable rx_err : int
    }
  let status = { rx_byte = 0; rx_msg = 0; rx_err = 0 }

  let msg_period = 1000 (** ms *)

(** Modem monitoring messages *)
  let send_msg =
    let rx_msg = ref 0 
    and rx_byte = ref 0 
    and start = Unix.gettimeofday () in
    fun () ->
      let dt = float msg_period /. 1000. in
      let t = int_of_float (Unix.gettimeofday () -. start) in
      let byte_rate = float (status.rx_byte - !rx_byte) /. dt
      and msg_rate = float (status.rx_msg - !rx_msg) /. dt in
      rx_msg := status.rx_msg;
      rx_byte := status.rx_byte;
      let vs = ["run_time", Pprz.Int t;
		"rx_bytes_rate", Pprz.Float byte_rate; 
		"rx_msgs_rate", Pprz.Float msg_rate;
		"rx_err", Pprz.Int status.rx_err;
		"rx_bytes", Pprz.Int status.rx_byte;
		"rx_msgs", Pprz.Int status.rx_msg
	      ] in
      Tm_Pprz.message_send "modem" "DOWNLINK_STATUS" vs;
      let vs = ["valim", Pprz.Float Modem.status.Modem.valim;
		"detected", Pprz.Int Modem.status.Modem.detected;
		"cd", Pprz.Int Modem.status.Modem.cd;
		"nb_err", Pprz.Int Modem.status.Modem.nb_err;
		"nb_byte", Pprz.Int Modem.status.Modem.nb_byte;
		"nb_msg", Pprz.Int Modem.status.Modem.nb_msg
	      ] in
      Tm_Pprz.message_send "modem" "MODEM_STATUS" vs

  let use_tele_message_modem = fun payload ->
    let buf = Serial.string_of_payload payload in
    status.rx_byte <- status.rx_byte + String.length buf;
    Debug.call 'P' (fun f -> fprintf f "use_pprz: %s\n" (Debug.xprint buf));
    use_tele_message payload


  let use_message =
    let buffer = ref "" in
    fun payload ->
      let msg = Serial.string_of_payload payload in
      Debug.call 'M' (fun f -> fprintf f "use_modem: %s\n" (Debug.xprint msg));
      match Modem.parse msg with
	None -> () (* Only internal modem data *)
      | Some data ->
	  (** Accumulate in a buffer *)
	  let b = !buffer ^ data in
	  Debug.call 'M' (fun f -> fprintf f "Pprz buffer: %s\n" (Debug.xprint b));
	  (** Parse as pprz message and ... *)
	  let x = PprzTransport.parse use_tele_message_modem b in
	  status.rx_err <- !PprzTransport.nb_err;
	  (** ... remove from the buffer the chars which have been used *)
	  buffer := String.sub b x (String.length b - x)
end (* PprzModem module *)


let parse_of_transport = function
    Pprz -> PprzTransport.parse use_tele_message
  | Modem -> 
      let module ModemTransport = Serial.Transport(Modem.Protocol) in
      ModemTransport.parse PprzModem.use_message
  | _ -> failwith "parse_of_transport: not yet"
    

let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in
  let baurate = ref "9600" in
  let transport = ref "pprz" in

  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "Ivy bus (%s)" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "Port (%s)" !port);
      "-transport", Arg.Set_string port, (sprintf "Transport (%s): modem,pprz,wavecard" !transport);
      "-s", Arg.Set_string baurate, (sprintf "Baudrate (%s)" !baurate)] in
  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  let transport = transport_of_string !transport in
  
  try
    let fd = Serial.opendev !port (Serial.speed_of_baudrate !baurate) in
    let ac = { fd=fd; transport=transport} in

    (* Listening *)
    let parse = parse_of_transport transport in
    
    let buffered_input = 
      match Serial.input parse with
	Serial.Closure f -> f in
    let cb = fun _ -> buffered_input fd; true in
    ignore (Glib.Io.add_watch [`IN] cb (GMain.Io.channel_of_descr fd));

    (** Listening on Ivy (FIXME: remove the ad hoc messages) *)
    ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" (get_fp ac));
    ignore (Ground_Pprz.message_bind "MOVE_WAYPOINT" (move_wp ac));
    ignore (Ground_Pprz.message_bind "SEND_EVENT" (send_event ac));
    ignore (Ground_Pprz.message_bind "DL_SETTING" (setting ac));
    ignore (Ground_Pprz.message_bind "JUMP_TO_BLOCK" (jump_block ac));
    ignore (Ground_Pprz.message_bind "RAW_DATALINK" (raw_datalink ac));

    (** Periodic tasks *)
    begin
      match transport with
	Modem ->
	  (** Sending periodically modem and downlink status messages *)
	  ignore (Glib.Timeout.add PprzModem.msg_period (fun () -> PprzModem.send_msg (); true))
      | _ -> ()
    end;


    (* Main Loop *)
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
  with
    exn -> fprintf stderr "%s\n" (Printexc.to_string exn)
