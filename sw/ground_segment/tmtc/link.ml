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

type transport =
    Modem
  | Pprz
  | Wavecard
  | XBee

type ground_device = { fd : Unix.file_descr; transport : transport ; baud_rate : int}

type airborne_device = 
    WavecardDevice of W.addr
  | XBeeDevice
  | Uart (** For HITL for example *)

let my_id = 0

let ios = int_of_string

let (//) = Filename.concat
let conf = Env.paparazzi_home // "conf"

let airborne_device = fun device addr ->
  match device with
    "WAVECARD" -> WavecardDevice (W.addr_of_string addr)
  | "XBEE" -> XBeeDevice
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
  }

let statuss = Hashtbl.create 3
let update_status = fun ac_id buf ->
  let status = 
    try Hashtbl.find statuss ac_id with Not_found ->
      let s = { last_rx_byte = 0; last_rx_msg = 0; rx_byte = 0; rx_msg = 0; rx_err = 0 } in
      Hashtbl.add statuss ac_id s;
      s in
  status.rx_byte <- status.rx_byte + String.length buf;
  status.rx_msg <- status.rx_msg + 1;
  status.rx_err <- !PprzTransport.nb_err

let status_msg_period = 1000 (** ms *)

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
  let conf_file = conf // "conf.xml" in
  List.fold_right (fun a r ->
    if ExtXml.tag_is a "aircraft" then
      let airframe_file = conf // ExtXml.attrib a "airframe" in
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
	  fprintf stderr "Error in '%s', file not found: %s\n" conf_file f;
	  r
      |	_ ->
	  fprintf stderr "Error in '%s', ignoring\n" airframe_file;
	  r
    else
      r)
    (Xml.children (Xml.parse_file conf_file))
    []

exception NotSendingToThis

let airborne_device = fun ac_id airframes device ->
  let ac_device = try Some (List.assoc ac_id airframes) with Not_found -> None in
  match ac_device, device with
    None, Pprz -> Uart
  | (Some (WavecardDevice _ as ac_device), Wavecard) |
    (Some (XBeeDevice as ac_device), XBee) ->
      ac_device
  | _ -> raise NotSendingToThis


let ground_id = 0

let use_tele_message = fun payload ->
  let buf = Serial.string_of_payload payload in
  Debug.call 'l' (fun f ->  fprintf f "pprz receiving: %s\n" (Debug.xprint buf));
  try
    let (msg_id, ac_id, values) = Tm_Pprz.values_of_payload payload in
    let msg = Tm_Pprz.message_of_id msg_id in
    Tm_Pprz.message_send (string_of_int ac_id) msg.Pprz.name values;
    update_status ac_id buf
  with
    _ -> ()


type priority = Null | Low | Normal | High

(******** Wavecard ******************************************************)
module Wc = struct
  type status = Ready | Busy
    
  let buffer_size = 5
  let null_buffer_entry = (Null, Unix.stdout, (W.ACK, ""))
  let priority_of = fun (p, _, _) -> p
  let buffer = (ref Ready, Array.create buffer_size null_buffer_entry)
  let timer = ref None
  let remove_timer = fun () ->
    match !timer with
      None -> ()
    | Some t -> GMain.Timeout.remove t

  let shift_buffer = fun b ->
    for i = 0 to buffer_size - 2 do (** A circular buf would be better *)
      b.(i) <- b.(i+1)
    done;
    b.(buffer_size-1) <- null_buffer_entry

  let rec repeat_send = fun fd cmd n ->
    W.send fd cmd;
    timer := Some (GMain.Timeout.add 300 (fun _ -> Debug.trace 'b' (sprintf "Retry %d" n); repeat_send fd cmd (n+1); false))

  let rec flush = fun n ->
    let status, b = buffer in
    if !status = Ready then
      let (priority, fd, cmd) = b.(0) in
      if priority <> Null then begin
	shift_buffer b;
	status := Busy;
	repeat_send fd cmd 0
      end

  let buffer_ready = fun () ->
    remove_timer ();
    let (status, b) = buffer in
    shift_buffer b;
    status := Ready;
    flush ()
	
	  	  
  let send_buffered = fun fd cmd priority ->
    let status, b = buffer in
    (** Set the message in the right place in the buffer *)
    let rec loop = fun i ->
      if i < buffer_size then
	if priority_of b.(i) >= priority
	then loop (i+1)
	  else begin
	    for j = i + 1 to buffer_size - 1 do (** Shift *)
	      b.(j) <- b.(j-1)
	    done;
	    Debug.trace 'b' (sprintf "Set in %d" i);
	    b.(i) <- (priority, fd, cmd)
	  end 
      else
	Debug.trace 'b' "Buffer full" in
    loop 0;
    flush ()

  let send = fun fd addr payload priority ->
    let data = W.addressed addr (Serial.string_of_payload payload) in
    send_buffered fd (W.REQ_SEND_MESSAGE, data) priority

  let ack_delay = 10 (* ms *)
  let send_ack = fun fd () ->
    Debug.trace 'w' (sprintf "%.2f send ACK" (Unix.gettimeofday ()));
    ignore (GMain.Timeout.add ack_delay (fun _ -> W.send fd (W.ACK, ""); false))
  let use_message = fun (com, data) ->
    match com with
      W.RECEIVED_FRAME ->
	use_tele_message (Serial.payload_of_string data)
    | W.RES_SEND_FRAME ->
	Debug.trace 'b' "RES_SEND_FRAME";
	ignore (GMain.Timeout.add 100 (fun _ -> buffer_ready (); false))
	
    | W.RES_READ_REMOTE_RSSI ->
	Tm_Pprz.message_send "link" "WC_RSSI" ["raw_level", Pprz.Int (Char.code data.[0])];
	Debug.call 'w' (fun f -> fprintf f "%.2f wv remote RSSI %d\n" (Unix.gettimeofday ()) (Char.code data.[0]));
	ignore (GMain.Timeout.add 100 (fun _ -> buffer_ready (); false))
    | W.RES_READ_RADIO_PARAM ->
	Ivy.send (sprintf "WC_ADDR %s" data);
	Debug.call 'w' (fun f -> fprintf f "wv local addr : %s\n" (Debug.xprint data));
    | W.ACK -> 
	Debug.trace 'w' (sprintf "%.2f wv ACK" (Unix.gettimeofday ()))
    | _ -> 
	Debug.call 'w' (fun f -> fprintf f "wv receiving: %02x %s\n" (W.code_of_cmd com) (Debug.xprint data));
	()

  let rssi_period = 5000 (** ms *)
  let req_rssi = fun device addr ->
    let data = W.addressed addr "" in
    send_buffered device.fd (W.REQ_READ_REMOTE_RSSI, data) Low

  let init = fun device rssi_id ->
    (** Set the wavecard in short wakeup mode *)
    let data = String.create 2 in
    data.[0] <- Char.chr (W.code_of_config_param W.WAKEUP_TYPE);
    data.[1] <- Char.chr (W.code_of_wakeup_type W.SHORT_WAKEUP);
(***          data.[0] <- Char.chr (W.code_of_config_param W.AWAKENING_PERIOD);
   data.[1] <- Char.chr 10; ***)
    W.send device.fd (W.REQ_WRITE_RADIO_PARAM,data);
    
    (* request own address *)
    let s = String.make 1 (char_of_int 5) in
    ignore (GMain.Timeout.add 1500 (fun _ -> W.send device.fd (W.REQ_READ_RADIO_PARAM, s); false));
    
    (** Ask for rssi if required *)
    if rssi_id >= 0 then begin
      match airborne_device rssi_id airframes device.transport with
	WavecardDevice addr ->
	  ignore (GMain.Timeout.add rssi_period (fun _ -> req_rssi device addr; true))
      | _ -> failwith (sprintf "Rssi not supported by A/C '%d'" rssi_id)
    end
	
end (** Wc module *)



module XB = struct (** XBee module *)
  let at_init_period = 2000 (* ms *)

  let switch_to_api = fun device ->
    let o = Unix.out_channel_of_descr device.fd in
(***    fprintf o "%s%!" (Xbee.at_set_my 255); ***)
    Debug.trace 'x' "config xbee";
    fprintf o "%s%!" (Xbee.at_set_baud_rate device.baud_rate);
    fprintf o "%s%!" Xbee.at_api_enable;
    fprintf o "%s%!" Xbee.at_exit;
    Debug.trace 'x' "end init xbee"

  let init = fun device ->
    Debug.trace 'x' "init xbee";
    let o = Unix.out_channel_of_descr device.fd in
    fprintf o "%s%!" Xbee.at_command_sequence;
    ignore (Glib.Timeout.add at_init_period (fun () -> switch_to_api device; false))

  let use_message = fun frame_data ->
    let frame_data = Serial.string_of_payload frame_data in
    Debug.trace 'x' (Debug.xprint frame_data);
    match Xbee.api_parse_frame frame_data with
      Xbee.Modem_Status x ->
	Debug.trace 'x' (sprintf "getting XBee status %d" x)
    | Xbee.AT_Command_Response (frame_id, comm, status, value) ->
	Debug.trace 'x' (sprintf "getting XBee AT command response: %d %s %d %s" frame_id comm status (Debug.xprint value))
    | Xbee.TX_Status (frame_id, status) ->
	Debug.trace 'x' (sprintf "getting XBee TX status: %d %d" frame_id status)
    | Xbee.RX_Packet_64 (addr64, rssi, options, data) ->
	Debug.trace 'x' (sprintf "getting XBee RX64: %Lx %d %d %s" addr64 rssi options (Debug.xprint data));
	use_tele_message (Serial.payload_of_string data)
    | Xbee.RX_Packet_16 (addr16, rssi, options, data) ->
	Debug.trace 'x' (sprintf "getting XBee RX16: from=%x %d %d %s" addr16 rssi options (Debug.xprint data));
	use_tele_message (Serial.payload_of_string data)


  let send = fun ac_id device rf_data ->
    let rf_data = Serial.string_of_payload rf_data in
    let frame_data = Xbee.api_tx16 ac_id rf_data in
    let packet = Xbee.Protocol.packet (Serial.payload_of_string frame_data) in
    let o = Unix.out_channel_of_descr device.fd in
    fprintf o "%s%!" packet;
    Debug.call 'y' (fun f -> fprintf f "link sending: (%s) %s\n" (Debug.xprint rf_data) (Debug.xprint packet));
end (** XBee module *)




let send = fun ac_id device ac_device payload priority ->
  match ac_device with
    Uart ->
      let o = Unix.out_channel_of_descr device.fd in
      let buf = Pprz.Transport.packet payload in
      Printf.fprintf o "%s" buf; flush o;
      Debug.call 'l' (fun f -> fprintf f "mm sending: %s\n" (Debug.xprint buf));
  | WavecardDevice addr ->
      Wc.send device.fd addr payload priority

  | XBeeDevice ->
      XB.send ac_id device payload


let cm_of_m = fun f -> Pprz.Int (truncate (100. *. f))

(** Got a FLIGHT_PARAM message and dispatch a ACINFO *)
let get_fp = fun device _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  List.iter 
    (fun (dest_id, _) ->
      if dest_id <> ac_id then (** Do not send to itself *)
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
	  NotSendingToThis -> ())
    airframes

(** Got a MOVE_WAYPOINT and send a MOVE_WP *)
let move_wp = fun device _sender vs ->
  Debug.call 'm' (fun f -> fprintf f "mm MOVE WAYPOINT\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  try
    let ac_device = airborne_device ac_id airframes device.transport in
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
    let s = Dl_Pprz.payload_of_values msg_id my_id vs in
    send ac_id device ac_device s High
  with
    NotSendingToThis -> ()

(** Got a SEND_EVENT, and send an EVENT *)
let send_event = fun device _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  try
    let ac_device = airborne_device ac_id airframes device.transport in
    let ev_id = Pprz.int_assoc "event_id" vs in
    let vs = ["event", Pprz.Int ev_id] in
    let msg_id, _ = Dl_Pprz.message_of_name "EVENT" in
    let s = Dl_Pprz.payload_of_values msg_id my_id vs in
    send ac_id device ac_device s High
  with
    NotSendingToThis -> ()
    

(** Got a DL_SETTING, and send an SETTING *)
let setting = fun device _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  try
    let ac_device = airborne_device ac_id airframes device.transport in
    let idx = Pprz.int_assoc "index" vs in
    let vs = ["index", Pprz.Int idx; "value", List.assoc "value" vs] in
    let msg_id, _ = Dl_Pprz.message_of_name "SETTING" in
    let s = Dl_Pprz.payload_of_values msg_id my_id vs in
    send ac_id device ac_device s High
  with
    NotSendingToThis -> ()

(** Got a JUMP_TO_BLOCK, and send an BLOCK *)
let jump_block = fun device _sender vs ->
  Debug.call 'j' (fun f -> fprintf f "mm JUMP_TO_BLOCK\n");
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  try
    let ac_device = airborne_device ac_id airframes device.transport in
    let block_id = Pprz.int_assoc "block_id" vs in
    let vs = ["block_id", Pprz.Int block_id] in
    let msg_id, _ = Dl_Pprz.message_of_name "BLOCK" in
    let s = Dl_Pprz.payload_of_values msg_id my_id vs in
    send ac_id device ac_device s High
  with
    NotSendingToThis -> ()

(** Got a RAW_DATALINK message *)
let raw_datalink = fun device _sender vs ->
  let ac_id = int_of_string (Pprz.string_assoc "ac_id" vs) in
  try
    let ac_device = airborne_device ac_id airframes device.transport in
    let m = Pprz.string_assoc "message" vs in
    for i = 0 to String.length m - 1 do
      if m.[i] = ';' then m.[i] <- ' '
    done;
    let msg_id, vs = Dl_Pprz.values_of_string m in
    let s = Dl_Pprz.payload_of_values msg_id my_id vs in
    send ac_id device ac_device s Normal
  with
    NotSendingToThis -> ()


module PprzModem = struct

  let msg_period = 1000 (** ms *)

(** Modem monitoring messages *)
  let send_msg = fun () ->
    let vs = ["valim", Pprz.Float Modem.status.Modem.valim;
	      "detected", Pprz.Int Modem.status.Modem.detected;
	      "cd", Pprz.Int Modem.status.Modem.cd;
	      "nb_err", Pprz.Int Modem.status.Modem.nb_err;
	      "nb_byte", Pprz.Int Modem.status.Modem.nb_byte;
	      "nb_msg", Pprz.Int Modem.status.Modem.nb_msg
	    ] in
    Tm_Pprz.message_send "modem" "MODEM_STATUS" vs
      
  let use_message =
    let buffer = ref "" in
    fun payload ->
      let msg = Serial.string_of_payload payload in
      Debug.call 'M' (fun f -> fprintf f "use_modem: %s\n" (Debug.xprint msg));
      match Modem.parse_payload payload with
	None -> () (* Only internal modem data *)
      | Some data ->
	  (** Accumulate in a buffer *)
	  let b = !buffer ^ data in
	  Debug.call 'M' (fun f -> fprintf f "Pprz buffer: %s\n" (Debug.xprint b));
	  (** Parse as pprz message and ... *)
	  let x = PprzTransport.parse use_tele_message b in
	  (** ... remove from the buffer the chars which have been used *)
	  buffer := String.sub b x (String.length b - x)
end (* PprzModem module *)


(*************** Audio *******************************************************)
module Audio = struct
  let use_data =
    let buffer = ref "" in
    fun data -> 
      let b = !buffer ^ data in
      let n = PprzTransport.parse use_tele_message b in
      buffer := String.sub b n (String.length b - n)
end







let parse_of_transport device = function
    Pprz -> 
      PprzTransport.parse use_tele_message
  | Modem -> 
      let module ModemTransport = Serial.Transport(Modem.Protocol) in
      ModemTransport.parse PprzModem.use_message
  | Wavecard ->
      fun buf -> Wavecard.parse buf ~ack:(Wc.send_ack device.fd) (Wc.use_message)
  | XBee ->
      let module XbeeTransport = Serial.Transport (Xbee.Protocol) in
      XbeeTransport.parse XB.use_message
    

let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in
  let baurate = ref "9600" in
  let transport = ref "pprz" in
  let uplink = ref false in
  let audio = ref false in
  let rssi_id = ref (-1) in
  let dtr = ref false in
  
  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
      "-rssi", Arg.Set_int rssi_id, (sprintf "<ac_id> Periodically requests rssi level from the distant wavecard");
      "-transport", Arg.Set_string transport, (sprintf "<transport> Available protocols are modem,pprz,wavecard and xbee. Default is %s" !transport);
      "-uplink", Arg.Set uplink, (sprintf "Uses the link as uplink also.");
      "-dtr", Arg.Set dtr, "Set serial DTR to false (aerocomm)";
      "-audio", Arg.Unit (fun () -> audio := true; port := "/dev/dsp"), (sprintf "Listen a modulated audio signal on <port>. Sets <port> to /dev/dsp (the -d option must used after this one if needed)");
      "-s", Arg.Set_string baurate, (sprintf "<baudrate>  Default is %s" !baurate)] in
  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  try    
    let transport = 
      match !transport with
	"modem" -> Modem
      | "pprz" -> Pprz
      | "wavecard" -> Wavecard
      | "xbee" -> XBee
      | x -> invalid_arg (sprintf "transport_of_string: %s" x)
    in

    (** Listen on a serial device or on multimon pipe or on audio *)
    let fd = 
      if !audio then
	Demod.init !port
      else
	if String.sub !port 0 4 = "/dev" then (* FIXME *)
	  Serial.opendev !port (Serial.speed_of_baudrate !baurate)
	else 
	  Unix.descr_of_in_channel (open_in !port)
    in

    if !dtr then
      Serial.set_dtr fd false;

    
    let device = { fd=fd; transport=transport; baud_rate=int_of_string !baurate } in

    (* Listening *)
    let buffered_input =
      let parse = parse_of_transport device transport in
      match Serial.input parse with
	Serial.Closure f -> f in
    let cb = 
      if !audio then
	fun _ ->
	  let (data_left, data_right) = Demod.get_data () in
	  Audio.use_data data_left;
	  true
      else
	fun _ -> buffered_input fd; true
    in
    ignore (Glib.Io.add_watch [`HUP] (fun _ -> exit 1)  (GMain.Io.channel_of_descr fd));
    ignore (Glib.Io.add_watch [`IN] cb (GMain.Io.channel_of_descr fd));


    if !uplink then begin
      (** Listening on Ivy (FIXME: remove the ad hoc messages) *)
      ignore (Ground_Pprz.message_bind "FLIGHT_PARAM" (get_fp device));
      ignore (Ground_Pprz.message_bind "MOVE_WAYPOINT" (move_wp device));
      ignore (Ground_Pprz.message_bind "SEND_EVENT" (send_event device));
      ignore (Ground_Pprz.message_bind "DL_SETTING" (setting device));
      ignore (Ground_Pprz.message_bind "JUMP_TO_BLOCK" (jump_block device));
      ignore (Ground_Pprz.message_bind "RAW_DATALINK" (raw_datalink device))
    end;


    (** Init and Periodic tasks *)
    begin
      ignore (Glib.Timeout.add status_msg_period (fun () -> send_status_msg (); true));
      match transport with
	Modem ->
	  (** Sending periodically modem and downlink status messages *)
	  ignore (Glib.Timeout.add PprzModem.msg_period (fun () -> PprzModem.send_msg (); true))
      | Wavecard ->
	  Wc.init device !rssi_id
      | XBee ->
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
  | exn -> fprintf stderr "%s\n" (Printexc.to_string exn)
