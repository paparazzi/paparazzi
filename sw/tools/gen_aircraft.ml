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
module U = Unix

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let conf_xml = paparazzi_conf // "conf.xml"
let modules_dir = paparazzi_conf // "modules"

let mkdir = fun d ->
  assert (Sys.command (sprintf "mkdir -p %s" d) = 0)

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
	Str.split pipe_regexp (ExtXml.attrib_or_default field "target" "ap|sim")
  with
	_ -> []

(** singletonize a sorted list *)
let rec singletonize = fun l ->
  match l with
	[] | [_] -> l
  | x :: ((y :: t) as yt) -> if x = y then singletonize yt else x :: singletonize yt

(** union of two lists *)
let union = fun l1 l2 ->
  let l = l1 @ l2 in
  let sl = List.sort compare l in
  singletonize sl

let union_of_lists = fun l ->
  let sl = List.sort compare (List.flatten l) in
  singletonize sl

(** [get_modules dir xml]
 * [dir] is the conf directory for modules, [xml] is the parsed airframe.xml *)
let get_modules = fun dir xml ->
  (* extract all "modules" sections *)
  let modules = List.map (fun x ->
	match String.lowercase (Xml.tag x) with
	  "modules" -> Xml.children x
	| _ -> []
	) (Xml.children xml) in
  (* flatten the list (result is a list of "load" xml nodes) *)
  let modules = List.flatten modules in
  (* build a list (file name, (xml, xml list of flags)) *)
  let extract = List.map (fun m ->
	match String.lowercase (Xml.tag m) with
	  "load" -> let file = dir // ExtXml.attrib m "name" in
		(file, (ExtXml.parse_file file, Xml.children m))
	| tag -> failwith (sprintf "Warning: tag load is undefined; found '%s'" tag)
	) modules in
  (* return a list of name and a list of pairs (xml, xml list) *)
  List.split extract

(** [get_targets_of_module xml] Returns the list of targets of a module *)
let get_targets_of_module = fun m ->
  let targets = List.map (fun x ->
	match String.lowercase (Xml.tag x) with
	  "makefile" -> targets_of_field x
	| _ -> []
  ) (Xml.children m) in
  (* return a singletonized list *)
  singletonize (List.sort compare (List.flatten targets))

(** [get_modules_dir xml] Returns the list of modules directories *)
let get_modules_dir = fun modules ->
  let dir = List.map (fun (m, _) -> try Xml.attrib m "dir" with _ -> ExtXml.attrib m "name") modules in
  singletonize (List.sort compare dir)

(**
   Search and dump the module section :
	 xml : the parsed airframe.xml
	 f   : makefile.ac
 **)
let dump_module_section = fun xml f ->
  (* get modules *)
  let (files, modules) = get_modules modules_dir xml in
  (* print modules directories and includes for all targets *)
  fprintf f "\n####################################################\n";
  fprintf f   "# modules makefile section\n";
  fprintf f   "####################################################\n";
  fprintf f "\n# include modules directory for all targets\n";
  (* get dir and target list *)
  let dir_list = get_modules_dir modules in
(**
  let target_list = union_of_lists (List.map (fun (m,_) -> get_targets_of_module m) modules) in
  List.iter (fun target -> fprintf f "%s.CFLAGS += -I modules -I arch/$(ARCH)/modules\n" target) target_list;
**)
  (** include modules directory for ALL targets and not just the defined ones **)
  fprintf f "$(TARGET).CFLAGS += -I modules -I arch/$(ARCH)/modules\n";
  List.iter (fun dir -> let dir_name = (String.uppercase dir)^"_DIR" in fprintf f "%s = modules/%s\n" dir_name dir) dir_list;
  (* parse each module *)
  List.iter (fun (m, flags) ->
	let name = ExtXml.attrib m "name" in
	let dir = try Xml.attrib m "dir" with _ -> name in
	let dir_name = (String.uppercase dir)^"_DIR" in
	(* get the list of all the targes for this module *)
	let module_target_list = get_targets_of_module m in
	(* print global flags as compilation defines and flags *)
	fprintf f "\n# makefile for module %s in modules/%s\n" name dir;
	List.iter (fun flag ->
      match String.lowercase (Xml.tag flag) with
        "define" ->
          let value = Xml.attrib flag "value"
          and name = Xml.attrib flag "name" in
          fprintf f "%s = %s\n" name value
      | "flag" | "param" ->
	      List.iter (fun target ->
		    let name = ExtXml.attrib flag "name"
		    and value = try "="^(Xml.attrib flag "value") with _ -> "" in
		    fprintf f "%s.CFLAGS += -D%s%s\n" target name value
	      ) module_target_list
      | _ -> ()
    ) flags;
	(* Look for makefile section *)
	List.iter (fun l ->
	  if ExtXml.tag_is l "makefile" then begin
		let targets = targets_of_field l in
		(* Look for defines, flags, files, ... *)
		List.iter (fun field ->
		  match String.lowercase (Xml.tag field) with
			"flag" ->
			  List.iter (fun target ->
				let value = try "="^(Xml.attrib field "value") with _ -> ""
				and name = Xml.attrib field "name" in
				let flag_type = match (ExtXml.attrib_or_default field "type" "define") with
					"define" | "D" -> "D"
				  | "include" | "I" -> "I"
				  | _ -> "D" in
				fprintf f "%s.CFLAGS += -%s%s%s\n" target flag_type name value
				) targets
		  | "file" ->
			  let name = Xml.attrib field "name" in
			  List.iter (fun target -> fprintf f "%s.srcs += $(%s)/%s\n" target dir_name name) targets
		  | "file_arch" ->
			  let name = Xml.attrib field "name" in
			  List.iter (fun target -> fprintf f "%s.srcs += arch/$(ARCH)/$(%s)/%s\n" target dir_name name) targets
		  | "file_hw" ->
			  let name = Xml.attrib field "name" in
			  List.iter (fun target -> fprintf f "%s.srcs += arch/$(ARCH)/$(%s)/%s\n" target dir_name name) targets
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
		  ) (Xml.children l)
	  end) (Xml.children m)
	) modules;
  (** returns a list of modules file name *)
  files

(**
	Search and dump the makefile sections
**)
let dump_makefile_section = fun xml makefile_ac airframe_infile print_if_loc_after ->
  List.iter (fun x ->
	if ExtXml.tag_is x "makefile" then begin
	  let located_before = ref true in
	  begin try
	located_before := not (String.compare (Xml.attrib x "location") "after" = 0)
	  with _ -> () end;
	  if  (not print_if_loc_after && !located_before) || (print_if_loc_after && not !located_before) then begin
	begin try
	  fprintf makefile_ac "\n# makefile target '%s'\n" (Xml.attrib x "target")
	with _ -> () end;
	match Xml.children x with
		  [Xml.PCData s] -> fprintf makefile_ac "%s\n" s
	| _ -> failwith (sprintf "Warning: wrong makefile section in '%s': %s\n" airframe_infile (Xml.to_string_fmt x))
	  end
	end)
	(Xml.children xml)

(** Firmware Children **)

let parse_subsystems = fun makefile_ac tag firmware ->
   match Xml.tag firmware with
	"subsystem" ->
		begin try
		  fprintf makefile_ac "# -subsystem: '%s' \n" (Xml.attrib firmware "name");
		  let has_subtype = ref false in
		  begin try
			has_subtype := not (String.compare (Xml.attrib firmware "type") "" = 0)
		  with _ -> () end;
		  let print_if_subsystem_define = (fun d ->
			if ExtXml.tag_is d "param" then begin
			  fprintf makefile_ac "%s = %s\n"
			  (String.uppercase(Xml.attrib d "name"))
			  (Xml.attrib d "value");
			end) in
		  List.iter print_if_subsystem_define (Xml.children firmware);
		  fprintf makefile_ac "include $(CFG_%s)/%s"
   		(String.uppercase(Xml.attrib tag "name"))
		(Xml.attrib firmware "name");
		  if !has_subtype then
			fprintf makefile_ac "_%s"
		  (Xml.attrib firmware "type");
		  fprintf makefile_ac ".makefile\n"
		with _ -> () end;
   | _ -> ()

let parse_targets = fun makefile_ac tag target ->
   match Xml.tag target with
   | "target" ->
		begin try
		  fprintf makefile_ac "\n###########\n# -target: '%s' \n" (Xml.attrib target "name");
		  fprintf makefile_ac "ifeq ($(TARGET), %s) \n" (Xml.attrib target "name");
  	  let print_if_subsystem = (fun c ->
			if ExtXml.tag_is c "param" then begin
			  fprintf makefile_ac "%s = %s\n"
				(String.uppercase(Xml.attrib c "name"))
				(Xml.attrib c "value")
			end) in
	  List.iter print_if_subsystem (Xml.children target);
		  let has_processor = ref false in
		  begin try
			has_processor := not (String.compare (Xml.attrib target "processor") "" = 0)
		  with _ -> () end;
		  if !has_processor then
			fprintf makefile_ac "BOARD_PROCESSOR = %s\n"
		  (Xml.attrib target "processor");
		  fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/boards/%s.makefile\n" (Xml.attrib target "board");
(**          fprintf makefile_ac "%s.ARCHDIR = $(ARCHI)\n"
		(Xml.attrib target "name") (Xml.attrib target "name")
		(Xml.attrib target "name") (Xml.attrib target "name")
		(Xml.attrib target "name") (Xml.attrib target "name");
**)          fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/autopilot/%s.makefile\n" (Xml.attrib tag "name");
  	  let print_if_subsystem = (fun d ->
			if ExtXml.tag_is d "define" then begin
			  let has_def_value = ref false in
			  begin try
				has_def_value := not (String.compare (Xml.attrib d "value") "" = 0)
			  with _ -> () end;
			  fprintf makefile_ac "%s.CFLAGS += -D%s"
				(Xml.attrib target "name")
				(Xml.attrib d "name");
			  if !has_def_value then
				fprintf makefile_ac "=%s" (Xml.attrib d "value");
			  fprintf makefile_ac "\n"
			end) in
	  List.iter print_if_subsystem (Xml.children target);
	  List.iter (parse_subsystems makefile_ac tag) (Xml.children target ); (** dump target  subsystems **)
	  List.iter (parse_subsystems makefile_ac tag) (Xml.children tag );    (** dump firware subsystems **)
		  fprintf makefile_ac "endif\n\n";
		with _ -> () end;
  | "define" ->
      let name = ExtXml.attrib target "name"
      and value = try "="^(Xml.attrib target "value") with _ -> "" in
      fprintf makefile_ac "$(TARGET).CFLAGS += -D%s%s\n" name value;
   | _ -> ()


(**
   Search and dump the firmware section
 **)
let dump_firmware_sections = fun xml makefile_ac ->
  List.iter (fun tag ->
	if ExtXml.tag_is tag "firmware" then begin
	  begin try
		fprintf makefile_ac "\n####################################################\n";
		fprintf makefile_ac   "# makefile firmware '%s' \n" (Xml.attrib tag "name");
		fprintf makefile_ac   "####################################################\n";
	List.iter (parse_targets makefile_ac tag) (Xml.children tag )
	  with _ -> () end;
	end)
	(Xml.children xml)



(** Extracts the makefile sections of an airframe file *)
let extract_makefile = fun airframe_file makefile_ac ->
  let xml = Xml.parse_file airframe_file in
  let f = open_out makefile_ac in

  fprintf f "# This file has been generated from %s by %s\n" airframe_file Sys.argv.(0);
  fprintf f "# Please DO NOT EDIT\n";

  (** Search and dump makefile sections that don't have a "location" attribute set to "after" *)
  dump_makefile_section xml f airframe_file false;
  (** Search and dump the firmware sections *)
  dump_firmware_sections xml f;
  (** Search and dump makefile sections that have a "location" attribute set to "after" *)
  dump_makefile_section xml f airframe_file true;

  (** Look for modules *)
  let module_files = dump_module_section  xml f in
  close_out f;
  module_files



let is_older = fun target_file dep_files ->
  not (Sys.file_exists target_file) ||
  let target_file_time = (U.stat target_file).U.st_mtime in
  let rec loop = function
	  [] -> false
	| f::fs ->
	target_file_time < (U.stat f).U.st_mtime ||
	loop fs in
  loop dep_files


let make_element = fun t a c -> Xml.Element (t,a,c)


(******************************* MAIN ****************************************)
let () =
  try
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
	let configuration =
	  make_element
	"configuration"
	[]
	[make_element "conf" [] [conf_aircraft]; Pprz.messages_xml ()] in
	let conf_aircraft_file = aircraft_conf_dir // "conf_aircraft.xml" in
	let f = open_out conf_aircraft_file in
	Printf.fprintf f "%s\n" (ExtXml.to_string_fmt configuration);
	close_out f;

	(** Computes and store a signature of the configuration *)
	let md5sum = Digest.to_hex (Digest.file conf_aircraft_file) in
	let md5sum_file = aircraft_conf_dir // "aircraft.md5" in
	(* Store only if different from previous one *)
	if not (Sys.file_exists md5sum_file
		  && md5sum = input_line (open_in md5sum_file)) then begin
		let f = open_out md5sum_file in
		Printf.fprintf f "%s\n" md5sum;
		close_out f;

		(** Save the configuration for future use *)
		let d = U.localtime (U.gettimeofday ()) in
		let filename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d_%s_%s.conf" (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday) (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec) md5sum aircraft in
		let d = Env.paparazzi_home // "var" // "conf" in
		mkdir d;
		let f = open_out (d // filename) in
		Printf.fprintf f "%s\n" (ExtXml.to_string_fmt configuration);
		close_out f end;

	let airframe_file = value "airframe" in

	let airframe_dir = Filename.dirname airframe_file in
	let var_airframe_dir = aircraft_conf_dir // airframe_dir in
	mkdir var_airframe_dir;
	assert (Sys.command (sprintf "cp %s %s" (paparazzi_conf // airframe_file) var_airframe_dir) = 0);

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

	(* Get TARGET env, needed to build modules.h according to the target *)
	let t = Printf.sprintf "TARGET=%s" (try Sys.getenv "TARGET" with _ -> "") in
	make "all_ac_h" t;
	make_opt "radio_ac_h" "RADIO" "radio";
	make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan"
  with
	Failure f ->
	  prerr_endline f;
	  exit 1
