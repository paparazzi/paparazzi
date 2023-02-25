(*
 * Log player
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

module Ground_Pprz = PprzLink.Messages(struct let name = "ground" end)
module Tm_Pprz = PprzLink.Messages(struct let name = "telemetry" end)

let (//) = Filename.concat
let replay_dir = Env.paparazzi_home // "var" // "replay"

let log = ref [||]

let write_xml = fun f xml ->
  let d = Filename.dirname f in
  ignore (Sys.command (sprintf "mkdir -p %s" d));
  let m = open_out f in
  fprintf m "%s" (Xml.to_string_fmt xml);
  close_out m


let store_conf = fun conf acs ->
  let l =
    List.fold_right (fun x r ->
      if ExtXml.tag_is x "aircraft" then
	if List.mem (ExtXml.attrib x "ac_id") acs then
	  let ac_name = ExtXml.attrib x "name" in
	  let ac_dir = replay_dir // "var" // "aircrafts" // ac_name in

	  let w = fun s ->
            let f = replay_dir // "conf" // ExtXml.attrib x s in
            write_xml f (ExtXml.child x s);

	    let f = ac_dir // "conf" // ExtXml.attrib x s in
	    write_xml f (ExtXml.child x s);
	    f in
	  ignore (w "airframe");
	  ignore (w "radio");
          let xml_settings =
            try
              ExtXml.child x "generated_settings"
            with _ ->
              Printf.printf "Replay: no settings for display\n%!";
              Xml.Element("settings",[],[])
          in
          write_xml (ac_dir // "settings.xml") xml_settings;
          write_xml (replay_dir // "settings.xml") xml_settings;
          (* test if flight plan is an original one or the dumped version *)
          let orig_fp = List.exists (fun e -> compare (Xml.tag e) "flight_plan" = 0) (Xml.children x) in
          if orig_fp then begin
	    let fp = w "flight_plan" in
	    (** We must "dump" the flight plan from the original one *)
	    ignore (Sys.command (sprintf "mkdir -p %s" ac_dir));
	    let dump = ac_dir // "flight_plan.xml" in
	    let c = sprintf "%s %s %s" Env.dump_fp fp dump in
	    if Sys.command c <> 0 then
	      failwith c;
          end
          else begin
            let f = ac_dir // "flight_plan.xml" in
            write_xml f (ExtXml.child x "dump");
          end;
	  Xml.Element ("aircraft", Xml.attribs x, [])::r
	else r
      else (** Keep ground section *)
	x::r)
      (Xml.children conf) [] in
  let orig_conf = Xml.Element ("conf", [], l) in
  write_xml (replay_dir // "conf" // "conf.xml") orig_conf

let store_messages = fun protocol ->
  write_xml (replay_dir // "var" // "messages.xml") protocol

let time_of = fun (t, _, _) -> t

let get_log_bounds = fun () ->
  let start = time_of !log.(0) in
  let end_ = time_of !log.(Array.length !log - 1) in
  (start, end_)



let load_log = fun xml_file ->
  let xml = ExtXml.parse_file xml_file in
  let data_file =  ExtXml.attrib xml "data_file" in

  let f = Ocaml_tools.find_file [Filename.dirname xml_file] data_file in
  let f = Ocaml_tools.open_compress f in
  let lines = ref [] in
  let acs = ref [] in
  try
    while true do
      let l = input_line f in
      try
	Scanf.sscanf l "%f %s %[^\n]"
	  (fun t ac m ->
	    lines := (t,ac,m):: !lines;
	    if not (List.mem ac !acs) then acs := ac :: !acs
	  )
      with
	_ -> ()
    done
  with
    End_of_file ->
      close_in f;
      log := Array.of_list (List.rev !lines);
      store_conf (ExtXml.child xml "conf") !acs;
      store_messages (ExtXml.child xml "protocol")


let timer = ref None
let was_running = ref false

let bus = ref Defivybus.default_ivy_bus
let port = ref "/dev/ttyUSB0"
let baudrate = ref "9600"
let hw_flow_control = ref false
let file_to_load = ref ""
let output_on_serial = ref false


let stop = fun () ->
  match !timer with
    None -> ()
  | Some t -> GMain.Timeout.remove t; timer := None


let index_of_time = fun log t ->
  let rec loop = fun a b ->
    if a >= b then a else
    let c = (a+b)/ 2 in
    if t <= time_of log.(c) then loop a c else loop (c+1) b in
  loop 0 (Array.length log - 1)

let run = fun serial_port log adj i0 speed no_gui ->
  let rec loop = fun i ->
    let (t, ac, m) = log.(i) in
    (* extract message name *)
    let name = Str.string_before m (Str.search_forward (Str.regexp " ") m 0) in
    (* continue if message is in telemetry class *)
    begin try
      let _ = Tm_Pprz.message_of_name name in
      Ivy.send (Printf.sprintf "replay%s %s" ac m);
      Ivy.send (Printf.sprintf "time%s %f" ac t);
      begin
        match serial_port with
          None -> ()
        | Some channel ->
            try
              let msg_id, vs = Tm_Pprz.values_of_string m in
              (* receiver_id lost in Ivy logs, but it was probably 0. *)
              let payload = Tm_Pprz.payload_of_values msg_id (int_of_string ac) 0 vs in
              let buf = Pprz_transport.Transport.packet payload in
              Debug.call 'o' (fun f -> fprintf f "%s\n" (Debug.xprint buf));
              fprintf channel "%s%!" buf
            with
              _ -> ()
      end;
    with _ -> (); end;
    (* also try to replay ground messages *)
    begin try
      let _ = Ground_Pprz.message_of_name name in
      Ivy.send (Printf.sprintf "replay_ground %s" m);
    with _ -> (); end;
    adj#set_value t;
    if i + 1 < Array.length log then begin
      let dt = time_of log.(i+1) -. t in
      timer := Some (GMain.Timeout.add ~ms:(truncate (1000. *. dt /. speed#value)) ~callback:(fun () -> loop (i+1);false))
    end else if no_gui then
      exit 0
  in
  loop i0


let play = fun ?(no_gui=false) serial_port adj speed ->
  stop ();
  if Array.length !log > 1 then
    run serial_port !log adj (index_of_time !log adj#value) speed no_gui


let init = fun () ->
  Arg.parse
    [ "-b", Arg.String (fun x -> bus := x), (sprintf "<ivy bus> Default is %s" !bus);
      "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
      "-o", Arg.Set output_on_serial, "Output binary messages on serial port";
      "-s", Arg.Set_string baudrate, (sprintf "<baudrate>  Default is %s" !baudrate);
      "-shfc",  Arg.Set hw_flow_control, "Enable UART hardware flow control (CTS/RTS)";]
    (fun x -> file_to_load := x)
    "Usage: ";

  if !file_to_load <> "" then
    load_log !file_to_load;


  let serial_port =
    if !output_on_serial then
      Some (Unix.out_channel_of_descr (Serial.opendev !port (Serial.speed_of_baudrate !baudrate) !hw_flow_control))
    else
      None in

  let adj = GData.adjustment
      ~value:0. ~lower:0. ~upper:1000.
      ~step_incr:0.5 ~page_incr:1.0 ~page_size:1.0 () in

  let speed = object
    val mutable v = 1. method value = v method set_value x = v <- x
  end in

  let world_update_time = fun _ vs ->
    speed#set_value (PprzLink.float_assoc "time_scale" vs)
  in

  ignore (Ground_Pprz.message_bind "WORLD_ENV" world_update_time);

  (serial_port, adj, speed)




let main = fun () ->
  Ivy.init "Paparazzi replay" "READY" (fun _ _ -> ());
  Ivy.start !bus;

  GMain.Main.main ()
