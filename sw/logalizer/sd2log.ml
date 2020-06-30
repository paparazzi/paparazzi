(*
 * Extraction of .log and .data file from a .TLM airborne SD file
 *
 * Copyright (C) 2009 ENAC, Pascal Brisset
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
let var_path = Env.paparazzi_home // "var"
let default_logs_path = var_path // "logs"
let conf_xml = ExtXml.parse_file (Env.paparazzi_home // "conf" // "conf.xml")


module Tm_Pprz = PprzLink.Messages (struct let name = "telemetry" end)
module Dl_Pprz = PprzLink.Messages (struct let name = "datalink" end)

module Parser = Protocol.Transport(Pprzlog_transport.Transport)

let run_command = fun com ->
  if Sys.command com <> 0 then begin
    fprintf stderr "Command '%s' failed\n" com;
    exit 1;
  end

let make_element = fun t a c -> Xml.Element (t,a,c)


let log_xml = fun ac_id ->
  let select = fun x -> ExtXml.int_attrib x "ac_id" = ac_id in
  let conf_ac =
    try
      ExtXml.child ~select conf_xml "aircraft"
    with
      Not_found ->
        failwith (sprintf "Error: A/C %d not found in conf.xml" ac_id)
  in
  let expanded_conf_ac = Env.expand_ac_xml ~raise_exception:false conf_ac in
  let expanded_conf =
    make_element (Xml.tag conf_xml) (Xml.attribs conf_xml) [expanded_conf_ac] in
  make_element
    "configuration"
    []
    [expanded_conf; PprzLink.messages_xml ()]

(* AWFUL : modules should be replaced by objects in pprz.ml
   ... or/and "datalink" and "telemetry" classes should be merged *)
let values_of_payload = fun log_msg ->
  match log_msg.Pprzlog_transport.source with
    0 -> Tm_Pprz.values_of_payload
  | 1 -> Dl_Pprz.values_of_payload
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)

let message_of_id = fun log_msg ->
  match log_msg.Pprzlog_transport.source with
    0 -> Tm_Pprz.message_of_id
  | 1 -> Dl_Pprz.message_of_id
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)

let string_of_message = fun log_msg ->
  match log_msg.Pprzlog_transport.source with
    0 -> Tm_Pprz.string_of_message
  | 1 -> Dl_Pprz.string_of_message
  | x -> failwith (sprintf "Unexpected source:%d in log msg" x)

(*let hex_of_array = function
  | PprzLink.Array array ->
      let n = Array.length array in
      (* One integer -> 2 chars *)
      let s = Bytes.create (2*n) in
      Array.iteri
        (fun i dec ->
          let hex = sprintf "%02x" (PprzLink.int_of_value array.(i)) in
          String.blit hex 0 s (2*i) 2)
        array;
      s
  | value ->
      failwith (sprintf "Error: expecting array, found %s" (PprzLink.string_of_value value))
*)

let xml_parse_compressed_file = fun file ->
    Xml.parse_in (Ocaml_tools.open_compress file)


(** Look for a file in var/ with md5 in filename. May raise Not_found.
    Format from gen_aircraft.ml : YY_MM_DD__HH_MM_SS_MD5_ACNAME.conf *)
let md5_ofs = 3*6+1
let md5_len = 32
let search_conf = fun md5 ->
  let dir = var_path // "conf" in
  let files = Sys.readdir dir in
  let rec loop = fun i ->
    if i < Array.length files then begin
      if String.length files.(i) > (md5_ofs + md5_len)
      && String.sub files.(i) md5_ofs md5_len = md5 then
        dir // files.(i)
      else
        loop (i+1)
    end else
      raise Not_found in
  loop 0


let convert_file = fun ?(output_dir=None) file ->
  let tmp_file = Filename.temp_file "tlm_from_sd" "data" in

  let f_in = open_in file
  and f_out = open_out tmp_file in

  let start_unix_time = ref None
  and md5 = ref ""
  and single_ac_id = ref (-1) in

  let logs_path = match output_dir with
  | None -> default_logs_path
  | Some x -> x
  in

  let use_payload = fun payload ->
    try
    let log_msg = Pprzlog_transport.parse payload in
    if log_msg.Pprzlog_transport.source > 1 then
      fprintf stderr "Invalid source (%d), skipping message\n" log_msg.Pprzlog_transport.source
    else
    let (msg_id, ac_id, vs) = values_of_payload log_msg log_msg.Pprzlog_transport.pprz_data in

    if log_msg.Pprzlog_transport.source = 0 && !single_ac_id < 0 then
      single_ac_id := ac_id;

    if ac_id <> !single_ac_id && log_msg.Pprzlog_transport.source = 0 then
      fprintf stderr "Discarding message with ac_id %d, previous one was %d\n%!" ac_id !single_ac_id
    else
      let msg_descr = message_of_id log_msg msg_id in
      let timestamp = Int32.to_float log_msg.Pprzlog_transport.timestamp /. 1e4 in
      fprintf f_out "%.4f %d %s\n" timestamp ac_id (string_of_message log_msg msg_descr vs);

      (** Looking for a date from a GPS message and a md5 from an ALIVE *)
      if log_msg.Pprzlog_transport.source = 0 then
        match msg_descr.PprzLink.name with
          "GPS" when !start_unix_time = None
              && ( PprzLink.int_assoc "mode" vs = 3
                 || PprzLink.int_assoc "week" vs > 0) ->
                     let itow = PprzLink.int_assoc "itow" vs / 1000
                     and week = PprzLink.int_assoc "week" vs in
                     let unix_time = Latlong.unix_time_of_tow ~week itow in
                     start_unix_time := Some (unix_time -. timestamp)
        | "ALIVE" when !md5 = "" ->
            md5 := PprzLink.hex_of_int_array (PprzLink.assoc "md5sum" vs)
        | _ -> ()
  with _ -> fprintf stderr "Parsing error, skipping message\n"
  in

  let parser = fun b -> Parser.parse use_payload (Bytes.to_string b) in
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
        | None ->
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
      fprintf stderr "Looking for %s conf...\n%!" !md5;
      let configuration =
        try xml_parse_compressed_file (search_conf !md5) with
          Not_found ->
            fprintf stderr "Not found...\n%!";
            if !single_ac_id >= 0 then begin
              fprintf stderr "Try to rebuild it for A/C %d ...\n%!" !single_ac_id;
              try log_xml !single_ac_id with _ ->
                fprintf stderr "Failure: A/C %d not found\n%!" !single_ac_id;
                Xml.PCData ""
            end else
              Xml.PCData "" in

      if configuration <> Xml.PCData "" then
        let log =
          ExtXml.subst_attrib "time_of_day" (string_of_float start_time)
            (ExtXml.subst_attrib "data_file" data_name configuration) in

        let f = open_out (logs_path // log_name) in
        output_string f (Xml.to_string_fmt log);
        close_out f;
        fprintf stderr "%s file produced\n%!" log_name
      else
        fprintf stderr "No .log produced\n";

      (** Save the original binary file *)
      let com = sprintf "cp %s %s" file (logs_path // tlm_name) in
      run_command com;
      fprintf stderr "%s file saved\n%!" tlm_name

let () =
  if Array.length Sys.argv = 2 then
    convert_file Sys.argv.(1)
  else if Array.length Sys.argv = 3 then
    convert_file ~output_dir:(Some Sys.argv.(2)) Sys.argv.(1)
  else begin
    fprintf stderr "Usage: %s <telemetry airborne file> [<output directory (default in Paparazzi logs folder)>]\n" Sys.argv.(0);
    exit 1;
  end
