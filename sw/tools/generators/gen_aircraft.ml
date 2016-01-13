(*
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

open Gen_common

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let default_conf_xml = paparazzi_conf // "conf.xml"

let mkdir = fun d ->
  assert (Sys.command (sprintf "mkdir -p %s" d) = 0)

(** Raises a Failure if an ID or a NAME appears twice in the conf *)
let check_unique_id_and_name = fun conf conf_xml ->
  let ids = Hashtbl.create 5 and names = Hashtbl.create 5 in
  ExtXml.iter_tag "aircraft"
    (fun x ->
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
      Hashtbl.add names name id
    ) conf

let configure_xml2mk = fun f xml ->
  let name = String.uppercase (ExtXml.attrib xml "name")
  and value = ExtXml.attrib xml "value" in
  fprintf f "%s = %s\n" name value

let define_xml2mk = fun f ?(target="$(TARGET)") ?(vpath=None) xml ->
  let name = Xml.attrib xml "name"
  and value = try "=" ^ Xml.attrib xml "value" with _ -> "" in
  let flag_type = match ExtXml.attrib_or_default xml "type" "define" with
  | "define" | "D" -> "D"
  | "include" | "I" -> "I" ^ (match vpath with Some vp -> vp ^ "/" | None -> "")
  | "raw" -> ""
  | _ -> "D" in
  let flag = sprintf "%s.CFLAGS += -%s%s%s" target flag_type name value in
  try
    (* TODO: add condition in xml syntax ? *)
    let cond = Xml.attrib xml "cond" in
    fprintf f "%s\n%s\nendif\n" cond flag
  with Xml.No_attribute _ -> fprintf f "%s\n" flag

let raw_xml2mk = fun f name xml ->
  match Xml.children xml with
  | [Xml.PCData s] -> fprintf f "%s\n" s
  | _ -> eprintf "Warning: wrong makefile section in '%s': %s\n"
        name (Xml.to_string_fmt xml)

let file_xml2mk = fun f ?(arch = false) dir_name target xml ->
  let name = Xml.attrib xml "name" in
  let dir_name = ExtXml.attrib_or_default xml "dir" ("$(" ^ dir_name ^ ")") in
  let fmt =
    if arch then format_of_string "%s.srcs += arch/$(ARCH)/%s/%s\n"
    else format_of_string "%s.srcs += %s/%s\n" in
  fprintf f fmt target dir_name name

let module_xml2mk = fun f target m ->
  if not (List.mem target m.targets) then () else
  let name = ExtXml.attrib m.xml "name" in
  let dir = try Xml.attrib m.xml "dir" with Xml.No_attribute _ -> name in
  let dir_name = String.uppercase dir ^ "_DIR" in
  (* print global flags as compilation defines and flags *)
  fprintf f "\n# makefile for module %s in modules/%s\n" name dir;
  List.iter (fun flag ->
    match String.lowercase (Xml.tag flag) with
    | "configure" -> configure_xml2mk f flag
    | "define" -> define_xml2mk f ~target flag
    | _ -> ()) m.param;
  (* Look for makefile section *)
  ExtXml.iter_tag "makefile"
    (fun section ->
      (* Look for defines, flags, files, ... if target is matching *)
      let section =
        let targets = Gen_common.targets_of_field section Env.default_module_targets in
        if Gen_common.test_targets target targets then section else Xml.Element ("makefile", [], [])
      in
      Xml.iter
      (fun field ->
          match String.lowercase (Xml.tag field) with
          | "configure" -> configure_xml2mk f field
          | "define" -> define_xml2mk f ~target ~vpath:m.vpath field
          | "flag" ->
              let value = Xml.attrib field "value"
              and name = Xml.attrib field "name" in
              fprintf f "%s.%s += -%s\n" target name value
          | "file" -> file_xml2mk f dir_name target field
          | "file_arch" -> file_xml2mk f ~arch:true dir_name target field
          | "raw" -> raw_xml2mk f name field
          | _ -> ()
        ) section
    ) m.xml

let modules_xml2mk = fun f target xml ->
  let modules = Gen_common.get_modules_of_airframe ~target xml in
  (* print modules directories and includes for all targets *)
  fprintf f "\n# include modules directory for all targets\n";
  (* get dir list *)
  let dir_list = Gen_common.get_modules_dir modules in
  (** include modules directory for ALL targets, not just the defined ones **)
  fprintf f "$(TARGET).CFLAGS += -Imodules -Iarch/$(ARCH)/modules\n";
  List.iter
    (fun dir -> fprintf f "%s_DIR = modules/%s\n" (String.uppercase dir) dir
    ) dir_list;
  (* add vpath for external modules *)
  List.iter
    (fun m -> match m.vpath with
    | Some vp -> fprintf f "VPATH += %s\n" vp
    | _ -> ()
    ) modules;
  fprintf f "\n";
  modules

(** Search and dump the makefile sections *)
let dump_makefile_section = fun xml makefile_ac airframe_infile location ->
  ExtXml.iter_tag "makefile"
    (fun x ->
      let loc = ExtXml.attrib_or_default x "location" "before" in
      match location, loc with
      | "before", "before" | "after", "after" ->
          fprintf makefile_ac "\n# raw makefile\n";
          raw_xml2mk makefile_ac airframe_infile x
      | _ -> ()
    ) xml

(** Firmware Children *)
let subsystem_xml2mk = fun f firmware s ->
  let name = ExtXml.attrib s "name"
  and s_type = try "_" ^ (Xml.attrib s "type") with Xml.No_attribute _ -> "" in
  fprintf f "\n# -subsystem: '%s'\n" name;
  let s_config, rest = ExtXml.partition_tag "configure" (Xml.children s) in
  let s_defines, _ = ExtXml.partition_tag "define" rest in
  (*List.iter (configure_xml2mk f) s_config;*)
  List.iter (fun def -> define_xml2mk f def) s_defines;
  (* include subsystem *) (* TODO test if file exists with the generator ? *)
  let s_name = name ^ s_type ^ ".makefile" in
  let s_dir = "CFG_" ^ String.uppercase (Xml.attrib firmware "name") in
  fprintf f "ifneq ($(strip $(wildcard $(%s)/%s)),)\n" s_dir s_name;
  fprintf f "\tinclude $(%s)/%s\n" s_dir s_name;
  fprintf f "else\n";
  fprintf f "\tinclude $(CFG_SHARED)/%s\n" s_name;
  fprintf f "endif\n"

let subsystem_configure_xml2mk = fun f s ->
  let s_config, _ = ExtXml.partition_tag "configure" (Xml.children s) in
  List.iter (configure_xml2mk f) s_config

let mod_or_subsys_xml2mk = fun f global_targets firmware target xml ->
  try
    let m = Gen_common.get_module xml global_targets in
    module_xml2mk f target m;
  with Gen_common.Subsystem _file -> subsystem_xml2mk f firmware xml

let parse_firmware = fun makefile_ac ac_xml firmware ->
  (* get the configures, targets, subsystems and defines for this firmware *)
  let config, rest = ExtXml.partition_tag "configure" (Xml.children firmware) in
  let targets, rest = ExtXml.partition_tag "target" rest in
  let mods, rest = ExtXml.partition_tag "module" rest in
  let subsystems, rest = ExtXml.partition_tag "subsystem" rest in
  let defines, _ = ExtXml.partition_tag "define" rest in
  (* iter on all targets *)
  List.iter (fun target ->
    (* get configures, defines and subsystems for this target *)
    let t_config, rest = ExtXml.partition_tag "configure" (Xml.children target) in
    let t_defines, rest = ExtXml.partition_tag "define" rest in
    let t_mods, rest = ExtXml.partition_tag "module" rest in
    let t_subsystems, _ = ExtXml.partition_tag "subsystem" rest in
    (* print makefile for this target *)
    let target_name = Xml.attrib target "name" in
    fprintf makefile_ac "\n###########\n# -target: '%s'\n" target_name;
    fprintf makefile_ac "ifeq ($(TARGET), %s)\n" target_name;
    let target_name = Xml.attrib target "name" in
    let modules = modules_xml2mk makefile_ac target_name ac_xml in
    begin (* Check for "processor" attribute *)
      try
        let proc = Xml.attrib target "processor" in
        fprintf makefile_ac "BOARD_PROCESSOR = %s\n" proc
      with Xml.No_attribute _ -> ()
    end;
    List.iter (configure_xml2mk makefile_ac) config;
    List.iter (configure_xml2mk makefile_ac) t_config;
    List.iter (subsystem_configure_xml2mk makefile_ac) subsystems;
    List.iter (subsystem_configure_xml2mk makefile_ac) t_subsystems;
    List.iter (subsystem_configure_xml2mk makefile_ac) mods;
    List.iter (subsystem_configure_xml2mk makefile_ac) t_mods;
    fprintf makefile_ac "\ninclude $(PAPARAZZI_SRC)/conf/boards/%s.makefile\n" (Xml.attrib target "board");
    fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/firmwares/%s.makefile\n" (Xml.attrib firmware "name");
    List.iter (fun def -> define_xml2mk makefile_ac def) defines;
    List.iter (fun def -> define_xml2mk makefile_ac def) t_defines;
    List.iter (module_xml2mk makefile_ac target_name) modules;
    List.iter (mod_or_subsys_xml2mk makefile_ac [] firmware target_name) mods;
    List.iter (subsystem_xml2mk makefile_ac firmware) t_subsystems;
    List.iter (subsystem_xml2mk makefile_ac firmware) subsystems;
    fprintf makefile_ac "\nendif # end of target '%s'\n\n" target_name
  ) targets


(** Search and dump the firmware section *)
let dump_firmware = fun f ac_xml firmware ->
  try
    fprintf f "\n####################################################\n";
    fprintf f   "# makefile firmware '%s'\n" (Xml.attrib firmware "name");
    fprintf f   "####################################################\n";
    parse_firmware f ac_xml firmware
  with Xml.No_attribute _ -> failwith "Warning: firmware name is undeclared"

let dump_firmware_sections = fun makefile_ac xml ->
  ExtXml.iter_tag "firmware"
    (fun tag -> dump_firmware makefile_ac xml tag) xml

(** Extracts the makefile sections of an airframe file *)
let extract_makefile = fun ac_id airframe_file makefile_ac ->
  let xml = Xml.parse_file airframe_file in
  let f = open_out makefile_ac in
  fprintf f "# This file has been generated by gen_aircraft from %s by %s\n"
    airframe_file Sys.executable_name;
  fprintf f "# Version %s\n" (Env.get_paparazzi_version ());
  fprintf f "# Please DO NOT EDIT\n";
  fprintf f "AC_ID=%s\n" ac_id;

  (** Search and dump makefile sections that have a "location" attribute set to "before" or no attribute *)
  dump_makefile_section xml f airframe_file "before";
  (** Search and dump the firmware sections *)
  dump_firmware_sections f xml;
  (** Search and dump makefile sections that have a "location" attribute set to "after" *)
  dump_makefile_section xml f airframe_file "after";
  close_out f

let is_older = fun target_file dep_files ->
  not (Sys.file_exists target_file) ||
    let target_file_time = (U.stat target_file).U.st_mtime in
    let rec loop = function
      | [] -> false
      | f :: fs -> target_file_time < (U.stat f).U.st_mtime || loop fs in
    loop dep_files


let make_element = fun t a c -> Xml.Element (t,a,c)


(******************************* MAIN ****************************************)
let () =
  try
    if Array.length Sys.argv < 2 || Array.length Sys.argv > 3 then
      failwith (sprintf "Usage: %s <Aircraft name> [conf.xml]" Sys.executable_name);
    let aircraft = Sys.argv.(1) in
    let conf_xml = if Array.length Sys.argv = 3 then Sys.argv.(2) else default_conf_xml in
    let conf = Xml.parse_file conf_xml in
    check_unique_id_and_name conf conf_xml;
    let aircraft_xml =
      try
        ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
      with
        Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft conf_xml)
    in

    let value = fun attrib -> ExtXml.attrib aircraft_xml attrib in

    let aircraft_dir = Env.paparazzi_home // "var" // "aircrafts" // aircraft in
    let aircraft_conf_dir = aircraft_dir // "conf" in

    let airframe_file = value "airframe" in
    let abs_airframe_file = paparazzi_conf // airframe_file in

    mkdir (Env.paparazzi_home // "var");
    mkdir (Env.paparazzi_home // "var" // "aircrafts");
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

    let target = try Sys.getenv "TARGET" with _ -> "" in
    let modules = Gen_common.get_modules_of_airframe ~target (Xml.parse_file abs_airframe_file) in
    (* normal settings *)
    let settings = try Env.filter_settings (value "settings") with _ -> "" in
    (* remove settings if not supported for the current target *)
    let settings = List.fold_left (fun l s -> if Gen_common.is_element_unselected ~verbose:true target modules s then l else l @ [s]) [] (Str.split (Str.regexp " ") settings) in
    (* update aircraft_xml *)
    let aircraft_xml = ExtXml.subst_attrib "settings" (String.concat " " settings) aircraft_xml in
    (* add modules settings *)
    let settings_modules = try Env.filter_settings (value "settings_modules") with _ -> "" in
    (* remove settings if not supported for the current target *)
    let settings_modules = List.fold_left (fun l s -> if Gen_common.is_element_unselected ~verbose:true target modules s then l else l @ [s]) [] (Str.split (Str.regexp " ") settings_modules) in
    (* update aircraft_xml *)
    let aircraft_xml = ExtXml.subst_attrib "settings_modules" (String.concat " " settings_modules) aircraft_xml in
    (* finally, concat all settings *)
    let settings = settings @ settings_modules in
    let settings = if List.length settings = 0 then
      begin
        fprintf stderr "\nInfo: No 'settings' attribute specified for A/C '%s', using 'settings/dummy.xml'\n\n%!" aircraft;
        "settings/dummy.xml"
      end
      else String.concat " " settings
    in

    (** Expands the configuration of the A/C into one single file *)
    let conf_aircraft = Env.expand_ac_xml aircraft_xml in
    let configuration =
      make_element
        "configuration" []
        [ make_element "conf" [] [conf_aircraft]; Pprz.messages_xml () ] in
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
      with Xml.No_attribute _ -> () in

    let temp_makefile_ac = Filename.temp_file "Makefile.ac" "tmp" in

    let () = extract_makefile (value "ac_id") abs_airframe_file temp_makefile_ac in

    (* Create Makefile.ac only if needed *)
    let makefile_ac = aircraft_dir // "Makefile.ac" in
    if is_older makefile_ac (abs_airframe_file ::(List.map (fun m -> m.file) modules)) then
      assert(Sys.command (sprintf "mv %s %s" temp_makefile_ac makefile_ac) = 0);

    (* Get TARGET env, needed to build modules.h according to the target *)
    let t = try Printf.sprintf "TARGET=%s" (Sys.getenv "TARGET") with _ -> "" in
    make "all_ac_h" t;
    make_opt "radio_ac_h" "RADIO" "radio";
    make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan"
  with Failure f ->
    prerr_endline f;
    exit 1
