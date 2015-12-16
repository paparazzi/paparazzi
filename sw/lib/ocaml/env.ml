(*
 * Configuration handling
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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

let (//) = Filename.concat
let space_regexp = Str.regexp "[ \t]+"
let make_element = fun t a c -> Xml.Element (t,a,c)

let paparazzi_src =
  try
    Sys.getenv "PAPARAZZI_SRC"
  with
      _ -> "/usr/share/paparazzi"

let paparazzi_home =
  try
    Sys.getenv "PAPARAZZI_HOME"
  with
      _ -> Filename.concat (Sys.getenv "HOME") "paparazzi"


let flight_plans_path = paparazzi_home // "conf" // "flight_plans"

let flight_plan_dtd = flight_plans_path // "flight_plan.dtd"

let icon_file = paparazzi_home // "data/pictures/penguin_icon.png"
let icon_gcs_file = paparazzi_home // "data/pictures/penguin_icon_gcs.png"
let icon_mes_file = paparazzi_home // "data/pictures/penguin_icon_msg.png"
let icon_rep_file = paparazzi_home // "data/pictures/penguin_icon_rep.png"
let icon_rtp_file = paparazzi_home // "data/pictures/penguin_icon_rtp.png"
let icon_log_file = paparazzi_home // "data/pictures/penguin_icon_log.png"
let icon_sim_file = paparazzi_home // "data/pictures/penguin_icon_sim.png"

let gconf_file = paparazzi_home // "conf" // "%gconf.xml"

let gcs_icons_path = paparazzi_home // "data" // "pictures" // "gcs_icons"
let gcs_default_icons_theme = "."

let get_gcs_icon_path = fun theme icon ->
  if Sys.file_exists (gcs_icons_path // theme // icon) then
    (* test if file exists *)
    gcs_icons_path // theme // icon
  else if Sys.file_exists (gcs_icons_path // icon) then
    (* else try default path *)
    gcs_icons_path // icon
  else
    (* or raise not found *)
    raise Not_found

let dump_fp = paparazzi_src // "sw" // "tools" // "generators" // "gen_flight_plan.out -dump"

let default_module_targets = "ap|sim|nps"

let filter_absolute_path = fun path ->
  Str.replace_first (Str.regexp (paparazzi_home // "conf/")) "" path


(* filter settings and keep the ones without brackets *)
let filter_settings = fun settings ->
  let sl = Str.split (Str.regexp "[ ]+") settings in
  let sl = List.filter (fun s -> not (s.[0] = '[' && s.[String.length s - 1] = ']')) sl in
  String.concat " " sl

(* filter on modules based on target *)
let filter_modules_target = fun module_file ->
  (* get TARGET env *)
  let target = try Sys.getenv "TARGET" with _ -> "" in
  (* look for a specific name after settings file (in case of modules) *)
  let split = Str.split (Str.regexp "~") module_file in
  let xml_file, name = match split with
    | [f; n] -> f, n
    | _ -> module_file, ""
  in
  let module_xml = Xml.parse_file xml_file in
  if Xml.tag module_xml = "module"
  then
    begin
      (* test if the module is loaded or not
       * and if a specific sub-settings is selected *)
      if List.exists (fun n ->
        let local_target = ExtXml.attrib_or_default n "target" default_module_targets
        and tag = Xml.tag n in
        if tag = "makefile" then
          Str.string_match (Str.regexp (".*"^target^".*")) local_target 0
        else false
        ) (Xml.children module_xml)
      then Xml.Element ("settings", [],
        List.filter (fun t ->
          Xml.tag t = "settings" && ExtXml.attrib_or_default t "name" "" = name)
        (Xml.children module_xml))
      else Xml.Element ("",[],[])
    end
  else module_xml


let expand_ac_xml = fun ?(raise_exception = true) ac_conf ->
  let prefix = fun s -> sprintf "%s/conf/%s" paparazzi_home s in
  let parse_file = fun ?(parse_filter=(fun x -> ExtXml.parse_file x)) a file ->
    try
      parse_filter file
    with
        Failure msg ->
          if raise_exception then
            failwith msg
          else begin
            prerr_endline msg;
            make_element "parse error" ["file",a; "msg", msg] []
          end in

  let parse = fun ?(pre_filter=(fun x -> x)) ?(parse_filter=(fun x -> ExtXml.parse_file x)) a ->
    List.map
      (fun filename -> parse_file ~parse_filter a (prefix filename))
      (Str.split space_regexp (pre_filter (ExtXml.attrib ac_conf a))) in

  let parse_opt = fun ?(pre_filter=(fun x -> x)) ?(parse_filter=(fun x -> ExtXml.parse_file x)) a ->
    try parse ~pre_filter ~parse_filter a with ExtXml.Error _ -> [] in

  (* dump expanded version of flight plan before parsing *)
  let parse_fp = fun a ->
    try
      (* get full path file name *)
      let fp = prefix (ExtXml.attrib ac_conf a) in
      if Sys.is_directory fp then raise Not_found;
      (* create a temporary dump file *)
      let dump = Filename.temp_file "fp_dump" ".xml" in
      (* set command then call it *)
      let c = sprintf "%s %s > %s" dump_fp fp dump in
      if Sys.command c <> 0 then
        begin
          Sys.remove dump;
          failwith c
        end;
      (* parse temp fp file and then remove it *)
      let fp_xml = parse_file a dump in
      Sys.remove dump;
      (* return Xml list *)
      [fp_xml]
    with _ -> []
  in

  let pervasives = parse "airframe" @ parse "telemetry" @ parse ~pre_filter:filter_settings "settings" in
  let optionals = parse_opt "radio" @ parse_fp "flight_plan" @ parse_opt ~pre_filter:filter_settings ~parse_filter:filter_modules_target "settings_modules"  @ pervasives in

  let children = Xml.children ac_conf@optionals in
  make_element (Xml.tag ac_conf) (Xml.attribs ac_conf) children

(* Run a command and return its results as a string. *)
let read_process command =
  let buffer_size = 2048 in
  let buffer = Buffer.create buffer_size in
  let string = String.create buffer_size in
  let in_channel = Unix.open_process_in command in
  let chars_read = ref 1 in
  while !chars_read <> 0 do
    chars_read := input in_channel string 0 buffer_size;
    Buffer.add_substring buffer string 0 !chars_read
  done;
  ignore (Unix.close_process_in in_channel);
  Buffer.contents buffer

let get_paparazzi_version = fun () ->
  try
    Str.replace_first (Str.regexp "[ \n]+$") "" (read_process (paparazzi_src ^ "/paparazzi_version"))
  with _ -> "UNKNOWN"
