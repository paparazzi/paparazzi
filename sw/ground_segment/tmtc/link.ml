(*
 * $Id$
 *
 * Copyright (C) 2004-2007 ENAC, Pascal Brisset, Antoine Drouin
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

(** Agent connecting a hardware modem, usually through USB/serial, with
  the Ivy sowtware bus.
*)

open Latlong
open Printf

(* Handlers for the modem and Ivy messages *)
module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)
module Ground_Pprz = Pprz.Messages (struct let name = "ground" end)
module Dl_Pprz = Pprz.Messages (struct let name = "datalink" end)
module PprzTransport = Serial.Transport (Pprz.Transport)

(* Modem transport layer *)
type transport =
    Pprz  (* Paparazzi protocol, with A/C id, message id and CRC *)
  | XBee  (* Maxstream protocol, API mode *)
let transport_of_string = function
    "pprz" -> Pprz
  | "xbee" -> XBee
  | x -> invalid_arg (sprintf "transport_of_string: %s" x)


type ground_device = {
    fd : Unix.file_descr; transport : transport ; baud_rate : int
  }

type airborne_device = 
    XBeeDevice
  | Uart (** For HITL for example *)

(* We assume here a single modem is used *)
let my_id = 0

let ios = int_of_string
let (//) = Filename.concat
let conf_dir = Env.paparazzi_home // "conf"

let airborne_device = fun device addr ->
  match device with
    "XBEE" -> XBeeDevice
  | "PPRZ" | "AEROCOMM" -> Uart
  | _ -> failwith (sprintf "Link: unknown datalink: %s" device)

let get_define = fun xml name ->
  let xml = ExtXml.child ~select:(fun d -> ExtXml.tag_is d "define" && ExtXml.attrib d "name" = name) xml "define" in
  ExtXml.attrib xml "value"


(*********** Monitoring *************************************************)
type status = {
    mutable last_rx_byte : int;
    mutable last_rx_msg : int;
    mutable rx_byte : int;
    mutable rx_msg : int;
    mutable rx_err : int;
    mutable ms_since_last_msg : int
  }

let statuss = Hashtbl.create 3
let dead_aircraft_time_ms = 5000
let update_status = fun ac_id buf_size ->
  let status = 
    try Hashtbl.find statuss ac_id with Not_found ->
      let s = { last_rx_byte = 0; last_rx_msg = 0; rx_byte = 0; rx_msg = 0; rx_err = 0; ms_since_last_msg = dead_aircraft_time_ms } in
      Hashtbl.add statuss ac_id s;
      s in
  status.rx_byte <- status.rx_byte + buf_size;
  status.rx_msg <- status.rx_msg + 1;
  status.rx_err <- !PprzTransport.nb_err;
  status.ms_since_last_msg <- 0

let status_msg_period = 1000 (** ms *)


let live_aircraft = fun ac_id ->
  try
    let s = Hashtbl.find statuss ac_id in
    s.ms_since_last_msg < dead_aircraft_time_ms
  with
    Not_found -> false

let send_status_msg =
  let start = Unix.gettimeofday () in
  fun () ->
    Hashtbl.iter (fun ac_id status ->
      let dt = float status_msg_period /. 1000. in
      let t = int_of_float (Unix.gettimeofday () -. start) in
      let byte_rate = float (status.rx_byte - status.last_rx_byte) /. dt
      and msg_rate = float (status.rx_msg - status.last_rx_msg) /. dt in
      status.last_rx_msg <- status.rx_msg;
      status.last_rx_byte <- status.rx_byte;
      status.ms_since_last_msg <- status.ms_since_last_msg + status_msg_period;
      let vs = ["run_time", Pprz.Int t;
		"rx_bytes_rate", Pprz.Float byte_rate; 
		"rx_msgs_rate", Pprz.Float msg_rate;
		"rx_err", Pprz.Int status.rx_err;
		"rx_bytes", Pprz.Int status.rx_byte;
		"rx_msgs", Pprz.Int status.rx_msg
	      ] in
      Tm_Pprz.message_send (string_of_int ac_id) "DOWNLINK_STATUS" vs)
      statuss 


let airframes =
  let conf_file = conf_dir // "conf.xml" in
  List.fold_right (fun a r ->
    if ExtXml.tag_is a "aircraft" then
      let airframe_file = conf_dir // ExtXml.attrib a "airframe" in
      try
	let airframe_xml = Xml.parse_file airframe_file in
	let dls = ExtXml.child ~select:(fun s -> Xml.attrib s "name" = "DATALINK") airframe_xml "section" in
	let device = get_define dls "DEVICE_TYPE"
	and addr = get_define dls "DEVICE_ADDRESS" in
	let dl = airborne_device device addr in
	(ios (ExtXml.attrib a "ac_id"), dl)::r
      with
	Not_found -> r
      |	Xml.File_not_found f ->
	  fprintf stderr "Error in '%s', file not found: %s\n%!" conf_file f;
	  r
      | exc ->
	  fprintf stderr "Error in '%s', ignoring: %s\n%!" airframe_file (Printexc.to_string exc);
	  r
    else
      r)
    (Xml.children (Xml.parse_file conf_file))
    []

exception NotSendingToThis

let airborne_device = fun ac_id airframes device ->
  let ac_device = try Some (List.assoc ac_id airframes) with Not_found -> None in
  match ac_device, device with
    (None, Pprz) | (Some Uart, Pprz) -> Uart
  | (Some (XBeeDevice as ac_device), XBee) ->
      ac_device
  | _ -> raise NotSendingToThis


let use_tele_message = fun ?raw_data_size payload ->
  let raw_data_size = match raw_data_size with None -> String.length (Serial.string_of_payload payload) | Some d -> d in
  let buf = Serial.string_of_payload payload in
  Debug.call 'l' (fun f ->  fprintf f "pprz receiving: %s\n" (Debug.xprint buf));
  try
    let (msg_id, ac_id, values) = Tm_Pprz.values_of_payload payload in
    let msg = Tm_Pprz.message_of_id msg_id in
    Tm_Pprz.message_send (string_of_int ac_id) msg.Pprz.name values;
    update_status ac_id raw_data_size
  with
    exc ->
      prerr_endline (Printexc.to_string exc);
      Debug.call 'W' (fun f ->  fprintf f "Warning, cannot use: %s\n" (Debug.xprint buf));


type priority = Null | Low | Normal | High

module Aerocomm = struct
  let set_command_mode = fun fd ->
    Serial.set_dtr fd true

  let set_data_mode = fun fd ->
    Serial.set_dtr fd false

  let write_destination_address = fun fd address ->
    assert (0 <= address && address <= 0xffffff);
    let o = Unix.out_channel_of_descr fd in
    let s = String.create 5 in
    s.[0] <- Char.chr 0xcc;
    s.[1] <- Char.chr 0x10;
    s.[2] <- Char.chr (address lsr 16);
    s.[3] <- Char.chr ((address lsr 8) land 0xff);
    s.[4] <- Char.chr (address land 0xff);
    fprintf o "%s%!" s
end


module XB = struct (** XBee module *)
  let nb_retries = ref 10
  let retry_delay = 200 (* ms *) 

  let at_init_period = 2000 (* ms *)

  let my_addr = ref 0x100

  let switch_to_api = fun device ->
    let o = Unix.out_channel_of_descr device.fd in
    Debug.trace 'x' "config xbee";
    fprintf o "%s%!" (Xbee.at_set_my !my_addr);
    fprintf o "%s%!" Xbee.at_api_enable;
    fprintf o "%s%!" Xbee.at_exit;
    Debug.trace 'x' "end init xbee"

  let init = fun device ->
    Debug.trace 'x' "init xbee";
    let o = Unix.out_channel_of_descr device.fd in
    ignore (Glib.Timeout.add at_init_period (fun () -> 
      fprintf o "%s%!" Xbee.at_command_sequence;
      ignore (Glib.Timeout.add at_init_period (fun () -> switch_to_api device; false));
      false))

  (* Array of sent packets for retry: (packet, nb of retries) *)
  let packets = Array.create 256 ("", -1)

  (* Frame id generation > 0 and < 256 *)
  let gen_frame_id = 
    let x = ref 0 in
    fun () -> 
      incr x;
      if !x >= 256 then
	x := 1;
      !x

  let oversize_packet = 4 (* Start + msb_len + lsb_len + cksum *)

  let use_message = fun device frame_data ->
    let frame_data = Serial.string_of_payload frame_data in
    Debug.trace 'x' (Debug.xprint frame_data);
    match Xbee.api_parse_frame frame_data with
      Xbee.Modem_Status x ->
	Debug.trace 'x' (sprintf "getting XBee status %d" x)
    | Xbee.AT_Command_Response (frame_id, comm, status, value) ->
	Debug.trace 'x' (sprintf "getting XBee AT command response: %d %s %d %s" frame_id comm status (Debug.xprint value))
    | Xbee.TX_Status (frame_id, status) ->
	Debug.trace 'x' (sprintf "getting XBee TX status: %d %d" frame_id status);
	if status = 1 then (* no ack, retry *)
	  let (packet, nb_prev_retries) = packets.(frame_id) in
	  if nb_prev_retries < !nb_retries then begin
	    packets.(frame_id) <- (packet, nb_prev_retries+1);
	    let o = Unix.out_channel_of_descr device.fd in
	    ignore (GMain.Timeout.add (10 + Random.int retry_delay)
	      (fun _ -> 
		fprintf o "%s%!" packet;
		Debug.call 'y' (fun f -> fprintf f "Resending (%d) %s\n" (nb_prev_retries+1) (Debug.xprint packet));
		false));
	  end
	  
    | Xbee.RX_Packet_64 (addr64, rssi, options, data) ->
	Debug.trace 'x' (sprintf "getting XBee RX64: %Lx %d %d %s" addr64 rssi options (Debug.xprint data));
	use_tele_message ~raw_data_size:(String.length frame_data + oversize_packet) (Serial.payload_of_string data)
    | Xbee.RX_Packet_16 (addr16, rssi, options, data) ->
	Debug.trace 'x' (sprintf "getting XBee RX16: from=%x %d %d %s" addr16 rssi options (Debug.xprint data));
	use_tele_message ~raw_data_size:(String.length frame_data + oversize_packet) (Serial.payload_of_string data)


  let send = fun ac_id device rf_data ->
    let rf_data = Serial.string_of_payload rf_data in
    let frame_id = gen_frame_id () in
    let frame_data = Xbee.api_tx16 ~frame_id ac_id rf_data in
    let packet = Xbee.Protocol.packet (Serial.payload_of_string frame_data) in

    (* Store the packet for further retry *)
    packets.(frame_id) <- (packet, 1);

    let o = Unix.out_channel_of_descr device.fd in
    fprintf o "%s%!" packet;
    Debug.call 'y' (fun f -> fprintf f "link sending (%d): (%s) %s\n" frame_id (Debug.xprint rf_data) (Debug.xprint packet));
end (** XBee module *)




let send = fun ac_id device ac_device payload priority ->
  match ac_device with
    Uart ->
      let o = Unix.out_channel_of_descr device.fd in
      let buf = Pprz.Transport.packet payload in
      Printf.fprintf o "%s" buf; flush o;
      Debug.call 'l' (fun f -> fprintf f "mm sending: %s\n" (Debug.xprint buf));
  | XBeeDevice ->
      XB.send ac_id device payload


let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f))

(** Got a FLIGHT_PARAM message and dispatch a ACINFO *)
let get_fp = fun device _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  List.iter 
    (fun (dest_id, _) ->
      if dest_id <> ac_id && live_aircraft dest_id then (** Do not send to itself *)
	try
	  Debug.trace 'b' (sprintf "ACINFO %d for %d" ac_id dest_id);
	  let ac_device = airborne_device dest_id airframes device.transport in
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
	  let s = Dl_Pprz.payload_of_values msg_id my_id vs in
	  send dest_id device ac_device s Low
	with
	  _NotSendingToThis -> ())
    airframes


(*************** Audio *******************************************************)
module Audio = struct
  let use_data =
    let buffer = ref "" in
    fun data -> 
      let b = !buffer ^ data in
      let n = PprzTransport.parse use_tele_message b in
      buffer := String.sub b n (String.length b - n)
end



let parser_of_device = fun device ->
  match device.transport with
    Pprz ->
      let use = fun s ->
	let raw_data_size = String.length (Serial.string_of_payload s) + 4(*stx,len,ck_a, ck_b*) in
	use_tele_message ~raw_data_size s in
      PprzTransport.parse use
  | XBee ->
      let module XbeeTransport = Serial.Transport (Xbee.Protocol) in
      XbeeTransport.parse (XB.use_message device)


let hangup = fun _ ->
  prerr_endline "Modem hangup. Exiting";
  exit 1


let forward_uplink = fun device ->
  let forwarder = fun name sender vs ->
    Debug.call 'f' (fun f -> fprintf f "forward %s\n" name);
    let ac_id = Pprz.int_assoc "ac_id" vs in
    try
      let ac_device = airborne_device ac_id airframes device.transport in
      let msg_id, _ = Dl_Pprz.message_of_name name in
      let s = Dl_Pprz.payload_of_values msg_id my_id vs in
      send ac_id device ac_device s High
    with
      NotSendingToThis -> () in
  let set_forwarder = fun name ->
    ignore (Dl_Pprz.message_bind name (forwarder name)) in

  set_forwarder "MOVE_WP";
  set_forwarder "SETTING";
  set_forwarder "BLOCK";
  set_forwarder "WIND_INFO"
  

(** Main *********************************************************************)
let () =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyUSB0" in
  let baudrate = ref "9600" in
  let transport = ref "pprz" in
  let uplink = ref true in
  let audio = ref false in
  let rssi_id = ref (-1)
  and aerocomm = ref false in
  
  (* Parse command line options *)
  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
      "-rssi", Arg.Set_int rssi_id, (sprintf "<ac_id> Periodically requests rssi level from the distant modem");
      "-xbee_addr", Arg.Set_int XB.my_addr, (sprintf "<my_addr> (%d)" !XB.my_addr);
      "-xbee_retries", Arg.Set_int XB.my_addr, (sprintf "<nb retries> (%d)" !XB.nb_retries);
      "-transport", Arg.Set_string transport, (sprintf "<transport> Available protocols are modem,pprz and xbee. Default is %s" !transport);
      "-uplink", Arg.Set uplink, (sprintf "Deprecated (now default)");
      "-nouplink", Arg.Clear uplink, (sprintf "Disables the uplink (from the ground to the aircraft).");
      "-dtr", Arg.Set aerocomm, "Set serial DTR to false (deprecated)";
      "-aerocomm", Arg.Set aerocomm, "Set serial Aerocomm data mode";
      "-audio", Arg.Unit (fun () -> audio := true; port := "/dev/dsp"), (sprintf "Listen a modulated audio signal on <port>. Sets <port> to /dev/dsp (the -d option must used after this one if needed)");
      "-s", Arg.Set_string baudrate, (sprintf "<baudrate>  Default is %s" !baudrate)] in
  Arg.parse options (fun _x -> ()) "Usage: ";

  (** Connect to Ivy bus *)
  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  try    
    let transport = transport_of_string !transport in
  
    (** Listen on audio input or on a serial device or on multimon pipe *)
    let on_serial_device = 
      String.length !port >= 4 && String.sub !port 0 4 = "/dev" in (* FIXME *)
    let fd = 
      if !audio then
	Demod.init !port
      else if on_serial_device then
	Serial.opendev !port (Serial.speed_of_baudrate !baudrate)
      else 
	Unix.openfile !port [Unix.O_RDWR] 0o640
    in

    (* Create the device object *)
    let baudrate = int_of_string !baudrate in
    let device = { fd=fd; transport=transport; baud_rate=baudrate } in

    (* The function to be called when data is available *)
    let read_fd = 
      if !audio then
	fun _io_event ->  (* Demodulation *)
	  let (data_left, _data_right) = Demod.get_data () in
	  Audio.use_data data_left;
	  true (* Returns true to be called again *)
      else (* Buffering and parsing *)
	let buffered_parser =
	  (* Get the specific parser for the given transport protocol *)
	  let parser = parser_of_device device in
	  (* Wrap the parser into the buffered bytes reader *)
	  match Serial.input parser with Serial.Closure f -> f in
	fun _io_event ->
	  begin
	    try buffered_parser fd with
	      exc -> prerr_endline (Printexc.to_string exc)
	  end;
	  true (* Returns true to be called again *)
    in
    ignore (Glib.Io.add_watch [`HUP] hangup (GMain.Io.channel_of_descr fd));
    ignore (Glib.Io.add_watch [`IN] read_fd (GMain.Io.channel_of_descr fd));

    if !uplink then begin
      ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" (get_fp device));
      forward_uplink device
    end;

    (** Init and Periodic tasks *)
    begin
      ignore (Glib.Timeout.add status_msg_period (fun () -> send_status_msg (); true));
      if !aerocomm then
	Aerocomm.set_data_mode fd;
      match transport with
	XBee ->
	  XB.init device
      | _ -> ()
    end;


    (* Main Loop *)
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
  with
    Xml.Error e -> prerr_endline (Xml.error e); exit 1
  | exn -> fprintf stderr "%s\n" (Printexc.to_string exn); exit 1
