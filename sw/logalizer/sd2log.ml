(*
 * $Id$
 *
 * Extraction of .log and .data file from a .TLM airborne SD file
 *  
 * Copyright (C) 2009 ENAC, Pascal Brisset
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
module U = Unix
let (//) = Filename.concat
let logs_path = Env.paparazzi_home // "var" // "logs"
let conf_xml = Xml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")


module Tm_Pprz = Pprz.Messages (struct let name = "telemetry" end)
module Dl_Pprz = Pprz.Messages (struct let name = "datalink" end)

module Parser = Serial.Transport(Logpprz.Transport)

let run_command = fun com ->
  if Sys.command com <> 0 then begin
    fprintf stderr "Command '%s' failed\n" com;
    exit 1;
  end

let make_element = fun t a c -> Xml.Element (t,a,c)


(** Copied from server.ml *)
let expand_ac_xml = fun ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" Env.paparazzi_home s in
  let parse = fun a ->
    let file = prefix (ExtXml.attrib ac_conf a) in
    try
      Xml.parse_file file
    with
      Xml.File_not_found _ ->
	prerr_endline (sprintf "File not found: %s" file);
	make_element "file_not_found" ["file",a] []
    | Xml.Error e ->
	let s = Xml.error e in
	prerr_endline (sprintf "Parse error in %s: %s" file s);
	make_element "cannot_parse" ["file",file;"error", s] [] in
  let fp = parse "flight_plan" in
  let af = parse "airframe" in
  let rc = parse "radio" in
  let children = Xml.children ac_conf@[fp; af; rc] in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children

let log_xml = fun ac_id timeofday data_file ->
  let select = fun x -> ExtXml.int_attrib x "ac_id" = ac_id in
  let conf_ac =
    try
      ExtXml.child ~select conf_xml "aircraft"
    with
      Not_found ->
	failwith (sprintf "Error: A/C %d not found in conf.xml" ac_id)
  in
  let expanded_conf_ac = expand_ac_xml conf_ac in
  let expanded_conf =
    make_element (Xml.tag conf_xml) (Xml.attribs conf_xml) [expanded_conf_ac] in
  make_element 
    "configuration"
    ["time_of_day", string_of_float timeofday; "data_file", data_file]
    [expanded_conf; Pprz.messages_xml ()]

(* AWFUL : modules should be replaced by objects in pprz.ml
   ... or/and "datalink" and "telemetry" classes should be merged *)
let values_of_payload = fun log_msg ->
  match log_msg.Logpprz.source with
    0 -> Tm_Pprz.values_of_payload
  | 1 -> Dl_Pprz.values_of_payload
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)

let message_of_id = fun log_msg ->
  match log_msg.Logpprz.source with
    0 -> Tm_Pprz.message_of_id
  | 1 -> Dl_Pprz.message_of_id
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)

let string_of_message = fun log_msg ->
  match log_msg.Logpprz.source with
    0 -> Tm_Pprz.string_of_message
  | 1 -> Dl_Pprz.string_of_message
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)


  
let convert_file = fun file ->
  let tmp_file = Filename.temp_file "tlm_from_sd" "data" in

  let f_in = open_in file
  and f_out = open_out tmp_file in

  let start_unix_time = ref None
  and single_ac_id = ref (-1) in

  let use_payload = fun payload ->
    let log_msg = Logpprz.parse payload in
    let (msg_id, ac_id, vs) = values_of_payload log_msg log_msg.Logpprz.pprz_data in

    if log_msg.Logpprz.source = 0 && !single_ac_id < 0 then
      single_ac_id := ac_id;

    if ac_id <> !single_ac_id && log_msg.Logpprz.source = 0 then
      fprintf stderr "Discarding message with ac_id %d, previous one was %d\n%!" ac_id !single_ac_id
    else
      let msg_descr = message_of_id log_msg msg_id in
      let timestamp = Int32.to_float log_msg.Logpprz.timestamp /. 1e4 in
      fprintf f_out "%.3f %d %s\n" timestamp ac_id (string_of_message log_msg msg_descr vs);
      
      (** Looking for a date from a GPS message *)
      if !start_unix_time = None
	  && log_msg.Logpprz.source = 0
	  && msg_descr.Pprz.name = "GPS"
	  && ( Pprz.int_assoc "mode" vs = 3 
	     || Pprz.int_assoc "week" vs > 0)  then
	let itow = Pprz.int_assoc "itow" vs / 1000
	and week = Pprz.int_assoc "week" vs in
	let unix_time = Latlong.unix_time_of_tow ~week itow in
	start_unix_time := Some (unix_time -. timestamp)
  in
  
  let parser = Parser.parse use_payload in
  let Serial.Closure reader = Serial.input parser in

  try
    while true do
      reader (U.descr_of_in_channel f_in)
    done
  with
    End_of_file ->
      close_in f_in;
      close_out f_out;

      prerr_endline "Renaming produced file ...";

      (* Rename the file according to the GPS time *)
      let start_time, mark =
	match !start_unix_time with
	  None -> 
	    fprintf stderr "Warning: not time found in GPS messages; using current date\n";
	    U.gettimeofday (), "_no_GPS" (* Not found, use now *)
	| Some u -> u, "" in

      let d = U.localtime start_time in
      let basename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d_SD%s" (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday) (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec) mark in
      let data_name = sprintf "%s.data" basename
      and log_name = sprintf "%s.log" basename
      and tlm_name = sprintf "%s.tlm" basename in

      (** Move the produced .data file *)
      let com = sprintf "mv %s %s" tmp_file (logs_path // data_name) in
      run_command com;
      fprintf stderr "%s file produced\n%!" data_name;

      (** Save the corresponding .log file *)
      if !single_ac_id >= 0 then
	let f = open_out (logs_path // log_name) in
	output_string f (Xml.to_string_fmt (log_xml !single_ac_id start_time data_name));
	close_out f;
	fprintf stderr "%s file produced\n%!" log_name
      else
	fprintf stderr "No telemetry message found: no .log produced\n";

      (** Save the original binary file *)
      let com = sprintf "cp %s %s" file (logs_path // tlm_name) in
      run_command com;
      fprintf stderr "%s file saved\n%!" tlm_name

let () =
  if Array.length Sys.argv = 2 then
    convert_file Sys.argv.(1)
  else begin
    fprintf stderr "Usage: %s <telemetry airborne file>\n" Sys.argv.(0);
    exit 1;
  end
