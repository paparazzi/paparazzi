(*
 * $Id$
 *
 * Call to Makefile.ac with the appropriate attributes from conf.xml
 *  
 * Copyright (C) 2003-2009 Pascal Brisset, Antoine Drouin, ENAC
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

let paparazzi_conf = Env.paparazzi_home // "conf" 
let conf_xml = paparazzi_conf // "conf.xml"
let modules_dir = paparazzi_conf // "modules"

let mkdir = fun d ->
  if not (Sys.file_exists d) then
    Unix.mkdir d 0o755

(** Raises a Failure if an ID or a NAME appears twice in the conf *)
let check_unique_id_and_name = fun conf ->
  let ids = Hashtbl.create 5
  and names = Hashtbl.create 5 in
  List.iter
    (fun x -> 
      if String.lowercase (Xml.tag x) = "aircraft" then 
	let id = ExtXml.attrib x "ac_id"
	and name = ExtXml.attrib x "name" in
	if Hashtbl.mem ids id then begin
	  let other_name = Hashtbl.find ids id in
	  failwith (sprintf "Error: A/C Id '%s' duplicated in %s (%s and %s)" id conf_xml name other_name)
	end;
	if Hashtbl.mem names name then begin
	  let other_id = Hashtbl.find names name in
	  failwith (sprintf "Error: A/C name '%s' duplicated in %s (ids %s and %s)" name conf_xml id other_id)
	end;
	Hashtbl.add ids id name;
	Hashtbl.add names name id)
    (Xml.children conf)



let pipe_regexp = Str.regexp "|"
let targets_of_field = fun field ->
  try 
    Str.split pipe_regexp (Xml.attrib field "target")
  with
    _ -> []


let get_modules = fun dir m ->
  match String.lowercase (Xml.tag m) with
    "load" -> dir // ExtXml.attrib m "name"
  | tag -> failwith (sprintf "Warning: tag load is undefined; found '%s'" tag)



(** Extracts the makefile section of an airframe file *)
let extract_makefile = fun airframe_file makefile_ac ->
  let xml = Xml.parse_file airframe_file in
  let f = open_out makefile_ac in

  fprintf f "# This file has been generated from %s by %s\n" airframe_file Sys.argv.(0);
  fprintf f "# Please DO NOT EDIT\n";

  (** Search and dump the makefile sections *)
  List.iter (fun x ->
    if ExtXml.tag_is x "makefile" then begin
      begin try
        fprintf f "\n# makefile target '%s'\n" (Xml.attrib x "target")
      with _ -> () end;
      match Xml.children x with
        [Xml.PCData s] -> fprintf f "%s\n" s
      | _ -> failwith (sprintf "Warning: wrong makefile section in '%s': %s\n" airframe_file (Xml.to_string_fmt x))
    end)
    (Xml.children xml);

  (** Look for modules *)
  let modules_exist = ref [] in (* Targets requring modules *)
  let files = ref [] in
  List.iter (fun x ->
    if ExtXml.tag_is x "modules" then
      let modules_names =List.map (get_modules modules_dir) (Xml.children x) in
      List.iter (fun name -> files := name :: !files) modules_names;
      let modules_list = List.map Xml.parse_file modules_names in
      List.iter (fun modul ->
        let name = ExtXml.attrib modul "name" in
        let dir_name = (String.uppercase name)^"_DIR" in
        fprintf f "\n# makefile for module %s\n" name;
        fprintf f "%s = $(PAPARAZZI_SRC)/sw/airborne/modules/%s\n" dir_name name;
        List.iter (fun l ->
          if ExtXml.tag_is l "makefile" then begin
            let targets = targets_of_field l in
            List.iter (fun t ->
              if not (List.mem t !modules_exist) then begin
                fprintf f "%s.srcs += $(ACINCLUDE)/modules.c\n" t;
                modules_exist := t :: !modules_exist
              end;
              fprintf f "%s.CFLAGS += -I $(%s)\n" t dir_name
            ) targets;
            List.iter (fun field ->
              match String.lowercase (Xml.tag field) with
                "flag" -> 
		              List.iter
		                (fun target -> 
		                  let value = try "="^(Xml.attrib field "value") with _ -> ""
		                  and name = Xml.attrib field "name" in
		                  fprintf f "%s.CFLAGS += -D%s%s\n" target name value)
		                targets
              | "file" -> 
		              let name = Xml.attrib field "name" in
		              List.iter (fun target -> fprintf f "%s.srcs += $(%s)/%s\n" target dir_name name) targets
              | "define" -> 
		            let value = Xml.attrib field "value"
		            and name = Xml.attrib field "name" in
		            fprintf f "%s = %s\n" name value
              | "raw" ->
                begin match Xml.children field with
                  [Xml.PCData s] -> fprintf f "%s\n" s
                  | _ -> fprintf stderr "Warning: wrong makefile section in module '%s'\n" name
                end
              | _ -> ()
            )
            (Xml.children l)
          end)
          (Xml.children modul))
	modules_list)
  (Xml.children xml);

  close_out f;
  !files



let is_older = fun target_file dep_files ->
  not (Sys.file_exists target_file) ||
  let target_file_time = (Unix.stat target_file).Unix.st_mtime in
  let rec loop = function
      [] -> false
    | f::fs ->
	target_file_time < (Unix.stat f).Unix.st_mtime ||
	loop fs in
  loop dep_files



(******************************* MAIN ****************************************)
let () =
  if Array.length Sys.argv <> 2 then
    failwith (sprintf "Usage: %s <A/C ident (conf.xml)>" Sys.argv.(0));
  let aircraft = Sys.argv.(1) in
  let conf = Xml.parse_file conf_xml in
  check_unique_id_and_name conf;
  let aircraft_xml =
    try
      ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
    with
      Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft conf_xml)
  in

  let value = fun attrib -> ExtXml.attrib aircraft_xml attrib in

  let aircraft_dir = Env.paparazzi_home // "var" // aircraft in
  let aircraft_conf_dir = aircraft_dir // "conf" in

  mkdir (Env.paparazzi_home // "var");
  mkdir aircraft_dir;
  mkdir (aircraft_dir // "fbw");
  mkdir (aircraft_dir // "autopilot");
  mkdir (aircraft_dir // "sim");
  mkdir aircraft_conf_dir;
  mkdir (aircraft_conf_dir // "airframes");
  mkdir (aircraft_conf_dir // "flight_plans");
  mkdir (aircraft_conf_dir // "radios");
  mkdir (aircraft_conf_dir // "settings");
  mkdir (aircraft_conf_dir // "telemetry");

  let settings = 
    try value "settings" with 
      _ -> 
	fprintf stderr "\nWARNING: No 'settings' attribute specified for A/C '%s', using 'settings/basic.xml'\n\n%!" aircraft;
	"settings/basic.xml" in

  (** Expands the configuration of the A/C into one single file *)
  let conf_aircraft = Env.expand_ac_xml aircraft_xml in
  let conf_aircraft_file = aircraft_conf_dir // "conf_aircraft.xml" in
  let f = open_out conf_aircraft_file in
  Printf.fprintf f "%s\n" (ExtXml.to_string_fmt conf_aircraft);
  close_out f;

  (** Computes and store a signature of the configuration *)
  let md5sum = Digest.to_hex (Digest.file conf_aircraft_file) in
  let md5sum_file = aircraft_conf_dir // "aircraft.md5" in
  let f = open_out md5sum_file in
  Printf.fprintf f "%s\n" md5sum;
  close_out f;

  let airframe_file = value "airframe" in
  
  (** Calls the Makefile with target and options *)
  let make = fun target options ->
    let c = sprintf "make -f Makefile.ac AIRCRAFT=%s AC_ID=%s AIRFRAME_XML=%s TELEMETRY=%s SETTINGS=\"%s\" MD5SUM=\"%s\" %s %s" aircraft (value "ac_id") airframe_file (value "telemetry") settings md5sum options target in
    begin (** Quiet is speficied in the Makefile *)
      try if Sys.getenv "Q" <> "@" then raise Not_found with
	Not_found -> prerr_endline c
    end;
    let returned_code = Sys.command c in
    if returned_code <> 0 then
      exit returned_code in

  (** Calls the makefile if the optional attribute is available *)
  let make_opt = fun target var attr ->
    try
      let value = Xml.attrib aircraft_xml attr in
      make target (sprintf "%s=%s" var value)
    with
      Xml.No_attribute _ -> () in

  let temp_makefile_ac = Filename.temp_file "Makefile.ac" "tmp" in
  let abs_airframe_file = paparazzi_conf // airframe_file in

  let modules_files = extract_makefile abs_airframe_file temp_makefile_ac in

  (* Create Makefile.ac only if needed *)
  let makefile_ac = aircraft_dir // "Makefile.ac" in
  if is_older makefile_ac (abs_airframe_file :: modules_files) then begin
    assert(Sys.command (sprintf "mv %s %s" temp_makefile_ac makefile_ac) = 0)
  end;

  make "all_ac_h" "";
  make_opt "radio_ac_h" "RADIO" "radio";
  make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan"
