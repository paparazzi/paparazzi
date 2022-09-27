(*
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
    the Ivy software bus.
*)


open Latlong
open Printf

(* Handlers for the modem and Ivy messages *)
module Tm_Pprz = PprzLink.Messages (struct let name = "telemetry" end)
module Ground_Pprz = PprzLink.Messages (struct let name = "ground" end)
module Dl_Pprz = PprzLink.Messages (struct let name = "datalink" end)
module PprzTransport = Protocol.Transport (Pprz_transport.Transport)

(* Modem transport layer *)
type transport =
  | Pprz  (* Paparazzi protocol, with A/C id, message id and CRC *)
  | XBee  (* Maxstream protocol, API mode *)
let transport_of_string = function
  | "pprz" -> Pprz
  | "xbee" -> XBee
  | x -> invalid_arg (sprintf "transport_of_string: %s" x)


type ground_device = {
  fd : Unix.file_descr; transport : transport ; baud_rate : int ; channel : int option
}

(* We assume here a single modem is used *)
let my_id = 0

(* Here we set the default id of the link*)
let link_id = ref (-1)
let red_link = ref false

(* enable broadcast messages by default *)
let ac_info = ref true

(* Listening on an UDP port *)
let udp = ref false
let udp_broadcast = ref false
let udp_broadcast_addr = if Os_calls.contains (Os_calls.os_name) "Darwin" then ref "224.255.255.255" else ref "127.255.255.255"
let udp_uplink_port = ref 4243

(* Enable trafic statistics on standard output *)
let gen_stat_trafic = ref false

let add_timestamp = ref None

let status_msg_period = ref 1000 (** ms *)
let ping_msg_period = ref 5000 (** ms  *)

(* Time (in ms) after which an aircraft is regarded as dead/off if no messages are received.
   If an aircraft is not live anymore, no uplink messages are sent.
   Set to a zero (or negative number) to disable this feature.
*)
let dead_aircraft_time_ms = ref 5000

let send_message_over_ivy = fun sender name vs ->
  let timestamp =
    match !add_timestamp with
      | None -> None
      | Some start_time -> Some (Unix.gettimeofday () -. start_time) in
  if !red_link then
    Tm_Pprz.message_send ?timestamp ~link_id:!link_id sender name vs
  else
    Tm_Pprz.message_send ?timestamp sender name vs

let send_ground_over_ivy = fun sender name vs ->
  let timestamp =
    match !add_timestamp with
      | None -> None
      | Some start_time -> Some (Unix.gettimeofday () -. start_time) in
  Ground_Pprz.message_send ?timestamp sender name vs


(*********** Monitoring *************************************************)
type status = {
  mutable last_rx_byte : int;
  mutable last_rx_msg : int;
  mutable rx_byte : int;
  mutable rx_msg : int;
  mutable rx_err : int;
  mutable tx_msg : int;
  mutable ms_since_last_msg : int;
  mutable last_ping : float; (* s *)
  mutable last_pong : float; (* s *)
  udp_peername : Unix.sockaddr option
}

let statuss = Hashtbl.create 3

let initial_status = {
  last_rx_byte = 0; last_rx_msg = 0;
  rx_byte = 0; rx_msg = 0; rx_err = 0;
  tx_msg = 0;
  ms_since_last_msg = !dead_aircraft_time_ms;
  last_ping = 0.; last_pong = 0.;
  udp_peername = None
}

let update_status = fun ?udp_peername ac_id buf_size is_pong ->
  if !gen_stat_trafic then
    Printf.printf "%.3f %d\n%!" (Unix.gettimeofday ()) buf_size;
  let status =
    try Hashtbl.find statuss ac_id with Not_found ->
      let s = { initial_status with udp_peername = udp_peername } in
      begin
        match s.udp_peername with
        | Some (Unix.ADDR_INET (peername, port)) ->
            Debug.call 'b' (fun f -> fprintf f "Adding AC %i with udp_peername %s port %i\n" ac_id (Unix.string_of_inet_addr peername) port)
        | _ -> Debug.call 'b' (fun f -> fprintf f "Adding AC %i\n" ac_id)
      end;
      Hashtbl.add statuss ac_id s;
      s in
  status.rx_byte <- status.rx_byte + buf_size;
  status.rx_msg <- status.rx_msg + 1;
  status.rx_err <- !PprzTransport.nb_err;
  status.ms_since_last_msg <- 0;
  if is_pong then
    status.last_pong <- Unix.gettimeofday ();;

let status_ping_diff = 500 (* ms *)

let live_aircraft = fun ac_id ->
  try
    let s = Hashtbl.find statuss ac_id in
    !dead_aircraft_time_ms <= 0 || s.ms_since_last_msg < !dead_aircraft_time_ms
  with
      Not_found -> false

let udp_peername = fun ac_id ->
  try
    (Hashtbl.find statuss ac_id).udp_peername
  with
      Not_found -> invalid_arg "udp_peername"

let last_udp_peername = ref (Unix.ADDR_UNIX "not initialized")
let udp_read = fun fd buf pos len ->
  let (n, sockaddr) = Unix.recvfrom fd buf pos len [] in
  last_udp_peername := sockaddr;
  n

let send_status_msg =
  let start = Unix.gettimeofday () in
  fun () ->
    Hashtbl.iter (fun ac_id status ->
      let dt = float !status_msg_period /. 1000. in
      let t = int_of_float (Unix.gettimeofday () -. start) in
      let byte_rate = float (status.rx_byte - status.last_rx_byte) /. dt
      and msg_rate = float (status.rx_msg - status.last_rx_msg) /. dt in
      status.last_rx_msg <- status.rx_msg;
      status.last_rx_byte <- status.rx_byte;
      let vs = ["ac_id", PprzLink.String (string_of_int ac_id);
                "link_id", PprzLink.String (string_of_int !link_id);
                "run_time", PprzLink.Int64 (Int64.of_int t);
                "rx_lost_time", PprzLink.Int64 (Int64.of_int (status.ms_since_last_msg / 1000));
                "rx_bytes", PprzLink.Int64 (Int64.of_int status.rx_byte);
                "rx_msgs", PprzLink.Int64 (Int64.of_int status.rx_msg);
                "rx_err", PprzLink.Int64 (Int64.of_int status.rx_err);
                "rx_bytes_rate", PprzLink.Float byte_rate;
                "rx_msgs_rate", PprzLink.Float msg_rate;
                "tx_msgs", PprzLink.Int64 (Int64.of_int status.tx_msg);
                "ping_time", PprzLink.Float (1000. *. (status.last_pong -. status.last_ping))
               ] in
      send_ground_over_ivy "link" "LINK_REPORT" vs)
      statuss

let update_ms_since_last_msg =
  fun () ->
    Hashtbl.iter (fun ac_id status ->
      status.ms_since_last_msg <- status.ms_since_last_msg + !status_msg_period / 3)
      statuss

let use_tele_message = fun ?udp_peername ?raw_data_size payload ->
  let raw_data_size = match raw_data_size with None -> String.length (Protocol.string_of_payload payload) | Some d -> d in
  let buf = Protocol.string_of_payload payload in
  Debug.call 'l' (fun f ->  fprintf f "pprz receiving: %s\n" (Debug.xprint buf));
  try
    let (header, values) = Tm_Pprz.values_of_payload payload in
    let ac_id = header.PprzLink.sender_id in
    let msg = Tm_Pprz.message_of_id header.PprzLink.message_id in
    send_message_over_ivy (string_of_int ac_id) msg.PprzLink.name values;
    update_status ?udp_peername ac_id raw_data_size (msg.PprzLink.name = "PONG")
  with
      exc ->
        prerr_endline (Printexc.to_string exc);
        Debug.call 'W' (fun f ->  fprintf f "Warning, cannot use: %s\n" (Debug.xprint buf));


type priority = Null | Low | Normal | High

module XB = struct (** XBee module *)
  let nb_retries = ref 10
  let retry_delay = 200 (* ms *)

  let at_init_period = 2000 (* ms *)

  let my_addr = ref 0x100

  let switch_to_api = fun device ->
    let o = Unix.out_channel_of_descr device.fd in
    Debug.trace 'x' "config xbee";
    fprintf o "%s%!" (Xbee_transport.at_set_my !my_addr);
    begin
      match device.channel with
          None -> ()
        | Some ch ->  fprintf o "%s%!" (Xbee_transport.at_set_channel ch);
    end;
    fprintf o "%s%!" Xbee_transport.at_api_enable;
    fprintf o "%s%!" Xbee_transport.at_exit;
    Debug.trace 'x' "end init xbee"

  let init = fun device ->
    Debug.trace 'x' "init xbee";
    let o = Unix.out_channel_of_descr device.fd in
    ignore (Glib.Timeout.add at_init_period (fun () ->
      fprintf o "%s%!" Xbee_transport.at_command_sequence;
      ignore (Glib.Timeout.add at_init_period (fun () -> switch_to_api device; false));
      false))

  (* Array of sent packets for retry: (packet, nb of retries) *)
  let packets = Array.make 256 ("", -1)

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
    let frame_data = Protocol.string_of_payload frame_data in
    Debug.trace 'x' (Debug.xprint frame_data);
    match Xbee_transport.api_parse_frame frame_data with
      | Xbee_transport.Modem_Status x ->
          Debug.trace 'x' (sprintf "getting XBee status %d" x)
      | Xbee_transport.AT_Command_Response (frame_id, comm, status, value) ->
        Debug.trace 'x' (sprintf "getting XBee AT command response: %d %s %d %s" frame_id comm status (Debug.xprint value))
      | Xbee_transport.TX_Status (frame_id,status) | Xbee_transport.TX868_Status (frame_id,status,_) ->
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

      | Xbee_transport.RX_Packet_64 (addr64, rssi, options, data) ->
        Debug.trace 'x' (sprintf "getting XBee RX64: %Lx %d %d %s" addr64 rssi options (Debug.xprint data));
        use_tele_message ~raw_data_size:(String.length frame_data + oversize_packet) (Protocol.payload_of_string data)
      | Xbee_transport.RX868_Packet (addr64, options, data) ->
        Debug.trace 'x' (sprintf "getting XBee868 RX: %Lx %d %s" addr64 options (Debug.xprint data));
        use_tele_message ~raw_data_size:(String.length frame_data + oversize_packet) (Protocol.payload_of_string data)
      | Xbee_transport.RX_Packet_16 (addr16, rssi, options, data) ->
        Debug.trace 'x' (sprintf "getting XBee RX16: from=%x %d %d %s" addr16 rssi options (Debug.xprint data));
        use_tele_message ~raw_data_size:(String.length frame_data + oversize_packet) (Protocol.payload_of_string data)


  let send = fun ?ac_id device rf_data ->
    let ac_id = match ac_id with None -> 0xffff | Some a -> a in
    let rf_data = Protocol.bytes_of_payload rf_data in
    let frame_id = gen_frame_id () in
    let frame_data =
      if !Xbee_transport.mode868 then
        Xbee_transport.api_tx64 ~frame_id (Int64.of_int ac_id) rf_data
      else
        Xbee_transport.api_tx16 ~frame_id ac_id rf_data in
    let packet = Xbee_transport.Transport.packet (Protocol.payload_of_string frame_data) in

    (* Store the packet for further retry *)
    packets.(frame_id) <- (packet, 1);

    let o = Unix.out_channel_of_descr device.fd in
    fprintf o "%s%!" packet;
    Debug.call 'y' (fun f -> fprintf f "link sending (%d): (%s) %s\n" frame_id (Debug.xprint (Bytes.to_string rf_data)) (Debug.xprint packet));
end (** XBee module *)


let local_broadcast_address =
  if Os_calls.contains (Os_calls.os_name) "Darwin" then
    (Unix.inet_addr_of_string "224.255.255.255")
  else
    (Unix.inet_addr_of_string "127.255.255.255")

let udp_send = fun fd payload peername ->
  let buf = Pprz_transport.Transport.packet payload in
  let len = String.length buf in
  let addr = if !udp_broadcast then (Unix.inet_addr_of_string !udp_broadcast_addr) else peername in
  Debug.call 'u' (fun f -> fprintf f "udp_send to %s port %i\n" (Unix.string_of_inet_addr addr) !udp_uplink_port);
  let sockaddr = Unix.ADDR_INET (addr, !udp_uplink_port) in
  let n = Unix.sendto fd (Bytes.of_string buf) 0 len [] sockaddr in
  assert (n = len)

let send = fun ac_id device payload _priority ->
  Debug.call 's' (fun f -> fprintf f "%d\n" ac_id);
  if live_aircraft ac_id then
    let _ = try
      let s = Hashtbl.find statuss ac_id in
      s.tx_msg <- s.tx_msg + 1;
      ()
    with Not_found -> () in
    match udp_peername ac_id with
      | Some (Unix.ADDR_INET (peername, _port)) ->
          udp_send device.fd payload peername
      | _ ->
        match device.transport with
          | Pprz ->
              let o = Unix.out_channel_of_descr device.fd in
              let buf = Pprz_transport.Transport.packet payload in
              Printf.fprintf o "%s" buf; flush o;
              Debug.call 's' (fun f -> fprintf f "mm sending: %s\n" (Debug.xprint buf));
          | XBee ->
            XB.send ~ac_id device payload


let broadcast = fun device payload _priority ->
  Hashtbl.iter (fun _ s -> s.tx_msg <- s.tx_msg + 1) statuss;
  if !udp then
    Hashtbl.iter (* Sending to all alive A/C *)
      (fun ac_id status ->
        if live_aircraft ac_id then
          match status.udp_peername with
              Some (Unix.ADDR_INET (peername, _port)) ->
                udp_send device.fd payload peername
            | _ -> ())
      statuss
  else
    match device.transport with
      | Pprz ->
          let o = Unix.out_channel_of_descr device.fd in
          let buf = Pprz_transport.Transport.packet payload in
          Printf.fprintf o "%s" buf; flush o;
          Debug.call 'l' (fun f -> fprintf f "mm sending: %s\n" (Debug.xprint buf));
      | XBee ->
        XB.send device payload


let parser_of_device = fun device ->
  match device.transport with
    | Pprz ->
        let use = fun s ->
          let raw_data_size = String.length (Protocol.string_of_payload s) + 4 (*stx,len,ck_a, ck_b*) in
          let udp_peername =
            if !udp then
              Some !last_udp_peername
            else
              None in
          use_tele_message ?udp_peername ~raw_data_size s in
        PprzTransport.parse use
    | XBee ->
      let module XbeeTransport = Protocol.Transport (Xbee_transport.Transport) in
      XbeeTransport.parse (XB.use_message device)


let hangup = fun _ ->
  prerr_endline "Modem hangup. Exiting";
  exit 1

(*************** Sending messages over link ***********************************)

let message_uplink = fun device ->
  let forwarder = fun name _sender vs ->
    Debug.call 'f' (fun f -> fprintf f "forward %s\n" name);
    let ac_id = PprzLink.int_assoc "ac_id" vs in
    let msg_id, _ = Dl_Pprz.message_of_name name in
    let s = Dl_Pprz.payload_of_values msg_id my_id ac_id vs in
    send ac_id device s High in
  let set_forwarder = fun name ->
    ignore (Dl_Pprz.message_bind name (forwarder name)) in

  let broadcaster = fun name _sender vs ->
    Debug.call 'f' (fun f -> fprintf f "broadcast %s\n" name);
    let msg_id, _ = Dl_Pprz.message_of_name name in
    let payload = Dl_Pprz.payload_of_values msg_id my_id PprzLink.broadcast_id vs in
    broadcast device payload Low in
  let set_broadcaster = fun name ->
    ignore (Dl_Pprz.message_bind name (broadcaster name)) in

  (* Set a forwarder or a broadcaster for all messages tagged in messages.xml *)
  Hashtbl.iter
    (fun _m_id msg ->
      match msg.PprzLink.link with
        | Some PprzLink.Forwarded -> set_forwarder msg.PprzLink.name
        | Some PprzLink.Broadcasted -> if !ac_info then set_broadcaster msg.PprzLink.name
        | _ -> ())
    Dl_Pprz.messages

let send_ping_msg = fun device ->
  Hashtbl.iter
    (fun ac_id status ->
      let msg_id, _ = Dl_Pprz.message_of_name "PING" in
      let s = Dl_Pprz.payload_of_values msg_id my_id ac_id [] in
      send ac_id device s High;
      status.last_ping <- Unix.gettimeofday ()
    )
    statuss


(** Main *********************************************************************)
let () =
  let ivy_bus = ref Defivybus.default_ivy_bus
  and port = ref "/dev/ttyUSB0"
  and baudrate = ref "9600"
  and channel = ref None
  and hw_flow_control = ref false
  and transport = ref "pprz"
  and uplink = ref true
  and udp_port = ref 4242 in

  (* Parse command line options *)
  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
      "-fg",  Arg.Set gen_stat_trafic, "Enable trafic statistics on standard output";
      "-noac_info", Arg.Clear ac_info, (sprintf "Disables AC traffic info (uplink).");
      "-nouplink", Arg.Clear uplink, (sprintf "Disables the uplink (from the ground to the aircraft).");
      "-s", Arg.Set_string baudrate, (sprintf "<baudrate>  Default is %s" !baudrate);
      "-ch", Arg.String (fun channel_string -> channel := Some (int_of_string channel_string)), "<channel>  Default does not change configuration.";
      "-hfc",  Arg.Set hw_flow_control, "Enable UART hardware flow control (CTS/RTS)";
      "-local_timestamp", Arg.Unit (fun () -> add_timestamp := Some (Unix.gettimeofday ())), "Add local timestamp to messages sent over ivy";
      "-transport", Arg.Set_string transport, (sprintf "<transport> Available protocols are modem,pprz,pprz2 and xbee. Default is %s" !transport);
      "-udp", Arg.Set udp, "Listen a UDP connection on <udp_port>";
      "-udp_port", Arg.Set_int udp_port, (sprintf "<UDP port> Default is %d" !udp_port);
      "-udp_uplink_port", Arg.Set_int udp_uplink_port, (sprintf "<UDP uplink port> Default is %d" !udp_uplink_port);
      "-udp_broadcast", Arg.Set udp_broadcast, "Broadcast on <udp_broadcast_addr>, e.g. for multiple simulated aircrafts";
      "-udp_broadcast_addr", Arg.Set_string udp_broadcast_addr, (sprintf "<addr> Broadcast address. Default is %s" !udp_broadcast_addr);
      "-uplink", Arg.Set uplink, (sprintf "Deprecated (now default)");
      "-xbee_addr", Arg.Set_int XB.my_addr, (sprintf "<my_addr> (%d)" !XB.my_addr);
      "-xbee_retries", Arg.Set_int XB.my_addr, (sprintf "<nb retries> (%d)" !XB.nb_retries);
      "-xbee_868", Arg.Set Xbee_transport.mode868, (sprintf "Enables the 868 protocol");
      "-redlink", Arg.Set red_link, (sprintf "Sets whether the link is a redundant link. Set this flag and the id flag to use multiple links");
      "-id", Arg.Set_int link_id, (sprintf "<id> Sets the link id. If multiple links are used, each must have a unique id. Default is %i" !link_id);
      "-status_period", Arg.Set_int status_msg_period, (sprintf "<period> Sets the period (in ms) of the LINK_REPORT status message. Default is %i" !status_msg_period);
      "-ping_period", Arg.Set_int ping_msg_period, (sprintf "<period> Sets the period (in ms) of the PING message sent to aircrafs. Default is %i" !ping_msg_period);
      "-ac_timeout", Arg.Set_int dead_aircraft_time_ms, (sprintf "<time> Sets the time (in ms) after which an aircraft is regarded as dead/off if no messages are received. Default is %ims, set to zero to disable." !ping_msg_period)
    ] in
  Arg.parse options (fun _x -> ()) "Usage: ";

  (** Connect to Ivy bus *)
  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  if (!link_id <> -1) && (not !red_link) then
    fprintf stderr "\nLINK WARNING: The link id was set to %i but the -redlink flag wasn't set. To use this link as a redundant link, set the -redlink flag.%!" !link_id;

  try
    let transport = transport_of_string !transport in

    (** Listen on a udp port or a serial device or on pipe *)
    let on_serial_device =
      String.length !port >= 4 && String.sub !port 0 4 = "/dev" in (* FIXME *)
    let fd =
      if !udp then
        begin
          let sockaddr = Unix.ADDR_INET (Unix.inet_addr_any, !udp_port)
          and socket = Unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in
          if !udp_broadcast then
            Unix.setsockopt socket Unix.SO_BROADCAST true;
            Unix.bind socket sockaddr;
            socket
        end
      else if on_serial_device then
          Serial.opendev !port (Serial.speed_of_baudrate !baudrate) !hw_flow_control
       else
          Unix.openfile !port [Unix.O_RDWR] 0o640
    in

    (* Create the device object *)
    let baudrate = int_of_string !baudrate in
    let device = { fd=fd; transport=transport; baud_rate=baudrate; channel= !channel } in

    (* The function to be called when data is available *)
    let read_fd =
      let buffered_parser =
        (* Get the specific parser for the given transport protocol *)
        let parser = fun b -> parser_of_device device (Bytes.to_string b) in
        let read = if !udp then udp_read else Unix.read in
        (* Wrap the parser into the buffered bytes reader *)
        match Serial.input ~read parser with Serial.Closure f -> f in
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
      message_uplink device
    end;

    (** Init and Periodic tasks *)
    begin
      ignore (Glib.Timeout.add !status_msg_period (fun () -> send_status_msg (); true));
      ignore (Glib.Timeout.add (!status_msg_period / 3) (fun () -> update_ms_since_last_msg (); true));
      if !uplink then begin
        let start_ping = fun () ->
          ignore (Glib.Timeout.add !ping_msg_period (fun () -> send_ping_msg device; true));
          false in
        ignore (Glib.Timeout.add status_ping_diff start_ping);
      end;
      match transport with
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
    | Xml.Error e -> prerr_endline (Xml.error e); exit 1
    | exn -> fprintf stderr "%s\n" (Printexc.to_string exn); exit 1
