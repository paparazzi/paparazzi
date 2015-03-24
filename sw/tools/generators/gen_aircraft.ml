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



(** [get_modules dir xml]
    * [dir] is the conf directory for modules, [xml] is the parsed airframe.xml *)
(*let get_modules = fun dir xml ->
  let modules = Gen_common.get_modules_of_airframe xml in
(* build a list (file name, (xml, xml list of flags)) *)
  let extract = List.map Gen_common.get_full_module_conf modules in
(* return a list of name and a list of pairs (xml, xml list) *)
  List.split extract*)

(**
   Search and dump the module section :
   xml : the parsed airframe.xml
   f   : makefile.ac
**)
let dump_module_section = fun xml f ->
  (* get modules *)
  let modules = Gen_common.get_modules_of_airframe xml in
  (* print modules directories and includes for all targets *)
  fprintf f "\n####################################################\n";
  fprintf f   "# modules makefile section\n";
  fprintf f   "####################################################\n";
  fprintf f "\n# include modules directory for all targets\n";
  (* get dir list *)
  let dir_list = Gen_common.get_modules_dir modules in
  (** include modules directory for ALL targets and not just the defined ones **)
  fprintf f "$(TARGET).CFLAGS += -Imodules -Iarch/$(ARCH)/modules\n";
  List.iter (fun dir -> let dir_name = (String.uppercase dir)^"_DIR" in fprintf f "%s = modules/%s\n" dir_name dir) dir_list;
  (* add vpath for external modules *)
  List.iter (fun m -> match m.vpath with Some vp -> fprintf f "VPATH += %s\n" vp | _ -> ()) modules;
  (* parse each module *)
  List.iter (fun m ->
    let name = ExtXml.attrib m.xml "name" in
    let dir = try Xml.attrib m.xml "dir" with _ -> name in
    let dir_name = (String.uppercase dir)^"_DIR" in
    (* get the list of all the targets for this module and concat the extra targets *)
    let module_target_list = Gen_common.get_targets_of_module m in
    (* print global flags as compilation defines and flags *)
    fprintf f "\n# makefile for module %s in modules/%s\n" name dir;
    List.iter (fun flag ->
      match String.lowercase (Xml.tag flag) with
          "configure" ->
            let value = Xml.attrib flag "value"
            and name = Xml.attrib flag "name" in
            fprintf f "%s = %s\n" name value
        | "define" ->
          List.iter (fun target ->
            let name = ExtXml.attrib flag "name"
            and value = try "="^(Xml.attrib flag "value") with _ -> "" in
            fprintf f "%s.CFLAGS += -D%s%s\n" target name value
          ) module_target_list
        | _ -> ()
    ) m.param;
    (* Look for makefile section *)
    List.iter (fun l ->
      if ExtXml.tag_is l "makefile" then begin
        (* add extra targets only if default is used *)
        let et = try ignore(Xml.attrib l "target"); [] with _ -> m.extra_targets in
        let targets = Gen_common.singletonize (
          Gen_common.targets_of_field l Env.default_module_targets @ et) in
        (* Look for defines, flags, files, ... *)
        List.iter (fun field ->
          match String.lowercase (Xml.tag field) with
              "configure" ->
                let value = Xml.attrib field "value"
                and name = Xml.attrib field "name" in
                fprintf f "%s = %s\n" name value
            | "define" ->
              List.iter (fun target ->
                let value = try "="^(Xml.attrib field "value") with _ -> ""
                and name = Xml.attrib field "name"
                and vpath = match m.vpath with Some vp -> vp^"/" | _ -> "" in
                let flag_type = match (ExtXml.attrib_or_default field "type" "define") with
                    "define" | "D" -> "D"
                  | "include" | "I" -> "I"^vpath
                  | "raw" -> ""
                  | _ -> "D" in
                try
                  let cond = Xml.attrib field "cond" in
                  fprintf f "%s\n" cond;
                  fprintf f "%s.CFLAGS += -%s%s%s\n" target flag_type name value;
                  fprintf f "endif\n";
                with _ ->
                  fprintf f "%s.CFLAGS += -%s%s%s\n" target flag_type name value
              ) targets
            | "flag" ->
              List.iter (fun target ->
                let value = Xml.attrib field "value"
                and name = Xml.attrib field "name" in
                fprintf f "%s.%s += -%s\n" target name value
              ) targets
            | "file" ->
              let name = Xml.attrib field "name" in
              let dir_name = ExtXml.attrib_or_default field "dir" ("$("^dir_name^")") in
              List.iter (fun target -> fprintf f "%s.srcs += %s/%s\n" target dir_name name) targets
            | "file_arch" ->
              let name = Xml.attrib field "name" in
              let dir_name = ExtXml.attrib_or_default field "dir" ("$("^dir_name^")") in
              List.iter (fun target -> fprintf f "%s.srcs += arch/$(ARCH)/%s/%s\n" target dir_name name) targets
            | "raw" ->
              begin match Xml.children field with
                  [Xml.PCData s] -> List.iter (fun target -> fprintf f "ifeq ($(TARGET), %s)\n%s\nendif\n" target s) targets
                | _ -> fprintf stderr "Warning: wrong makefile section in module '%s'\n" name
              end
            | _ -> ()
        ) (Xml.children l)
      end) (Xml.children m.xml)
  ) modules;
  (** returns a list of modules file name *)
  List.map (fun m -> m.file) modules

(**
   Search and dump the makefile sections
**)
let dump_makefile_section = fun xml makefile_ac airframe_infile location ->
  List.iter (fun x ->
    if ExtXml.tag_is x "makefile" then begin
      let loc = ExtXml.attrib_or_default x "location" "before" in
      match (location, loc) with
          ("before", "before") | ("after", "after") ->
            fprintf makefile_ac "\n# raw makefile\n";
            begin match Xml.children x with
                [Xml.PCData s] -> fprintf makefile_ac "%s\n" s
              | _ -> failwith (sprintf "Warning: wrong makefile section in '%s': %s\n" airframe_infile (Xml.to_string_fmt x))
            end
        | (_, _) -> ()
    end)
    (Xml.children xml)

(**
   * Firmware Children
   * **)

(* print a configure (firmware) *)
let print_firmware_configure = fun f p ->
  let name = (String.uppercase (Xml.attrib p "name"))
  and value = (Xml.attrib p "value") in
  fprintf f "%s = %s\n" name value

(* print a define (firmware) *)
let print_firmware_define = fun f d ->
  let name = ExtXml.attrib d "name"
  and value = try "="^(Xml.attrib d "value") with _ -> "" in
  fprintf f "$(TARGET).CFLAGS += -D%s%s\n" name value

(* print a subsystem (firmware) *)
let print_firmware_subsystem = fun f firmware s ->
  let name = ExtXml.attrib s "name"
  and s_type = try "_"^(Xml.attrib s "type") with _ -> "" in
  fprintf f "# -subsystem: '%s'\n" name;
  (* print config *)
  let s_config = List.filter (fun x -> ExtXml.tag_is x "configure") (Xml.children s) in
  List.iter (print_firmware_configure f) s_config;
  (* print defines *)
  let s_defines = List.filter (fun x -> ExtXml.tag_is x "define") (Xml.children s) in
  List.iter (print_firmware_define f) s_defines;
  (* include subsystem *) (* TODO test if file exists with the generator ? *)
  let s_name = name^s_type^".makefile" in
  let s_dir = "CFG_"^(String.uppercase (Xml.attrib firmware "name")) in
  fprintf f "ifneq ($(strip $(wildcard $(%s)/%s)),)\n" s_dir s_name;
  fprintf f "\tinclude $(%s)/%s\n" s_dir s_name;
  fprintf f "else\n";
  fprintf f "\tinclude $(CFG_SHARED)/%s\n" s_name;
  fprintf f "endif\n"

let parse_firmware = fun makefile_ac firmware ->
  (* get the list of configure for this firmware *)
  let config = List.filter (fun x -> ExtXml.tag_is x "configure") (Xml.children firmware) in
  (* get the list of targets for this firmware *)
  let targets = List.filter (fun x -> ExtXml.tag_is x "target") (Xml.children firmware) in
  (* get the list of subsystems for this firmware *)
  let subsystems = List.filter (fun x -> ExtXml.tag_is x "subsystem") (Xml.children firmware) in
  (* get the list of defines for this firmware *)
  let defines = List.filter (fun x -> ExtXml.tag_is x "define") (Xml.children firmware) in
  (* iter on all targets *)
  List.iter (fun target ->
    (* get the list of configure for this target *)
    let t_config = List.filter (fun x -> ExtXml.tag_is x "configure") (Xml.children target) in
    (* get the list of defines for this target *)
    let t_defines = List.filter (fun x -> ExtXml.tag_is x "define") (Xml.children target) in
    (* get the list of subsystems for this target *)
    let t_subsystems = List.filter (fun x -> ExtXml.tag_is x "subsystem") (Xml.children target) in
    (* print makefile for this target *)
    fprintf makefile_ac "\n###########\n# -target: '%s'\n" (Xml.attrib target "name");
    fprintf makefile_ac "ifeq ($(TARGET), %s)\n" (Xml.attrib target "name");
    begin (* Check for "processor" attribute *)
      try fprintf makefile_ac "BOARD_PROCESSOR = %s\n" (Xml.attrib target "processor")
      with _ -> ()
    end;
    List.iter (print_firmware_configure makefile_ac) config;
    List.iter (print_firmware_configure makefile_ac) t_config;
    fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/boards/%s.makefile\n" (Xml.attrib target "board");
    fprintf makefile_ac "include $(PAPARAZZI_SRC)/conf/firmwares/%s.makefile\n" (Xml.attrib firmware "name");
    List.iter (print_firmware_define makefile_ac) defines;
    List.iter (print_firmware_define makefile_ac) t_defines;
    List.iter (print_firmware_subsystem makefile_ac firmware) t_subsystems;
    List.iter (print_firmware_subsystem makefile_ac firmware) subsystems;
    fprintf makefile_ac "endif\n\n"
  ) targets


(**
   Search and dump the firmware section
**)
let dump_firmware_sections = fun xml makefile_ac ->
  List.iter (fun tag ->
    if ExtXml.tag_is tag "firmware" then begin
      try
        fprintf makefile_ac "\n####################################################\n";
        fprintf makefile_ac   "# makefile firmware '%s'\n" (Xml.attrib tag "name");
        fprintf makefile_ac   "####################################################\n";
        parse_firmware makefile_ac tag
      with _ -> failwith "Warning: firmware name is undeclared"
    end)
    (Xml.children xml)



(** Extracts the makefile sections of an airframe file *)
let extract_makefile = fun ac_id airframe_file makefile_ac ->
  let xml = Xml.parse_file airframe_file in
  let f = open_out makefile_ac in

  fprintf f "# This file has been generated by gen_aircraft from %s by %s\n" airframe_file Sys.argv.(0);
  fprintf f "# Please DO NOT EDIT\n";
  fprintf f "AC_ID=%s\n" ac_id;

  (** Search and dump makefile sections that have a "location" attribute set to "before" or no attribute *)
  dump_makefile_section xml f airframe_file "before";
  (** Search and dump the firmware sections *)
  dump_firmware_sections xml f;
  (** Search and dump makefile sections that have a "location" attribute set to "after" *)
  dump_makefile_section xml f airframe_file "after";

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
    if Array.length Sys.argv < 2 || Array.length Sys.argv > 3 then
      failwith (sprintf "Usage: %s <Aircraft name> [conf.xml]" Sys.argv.(0));
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

    let settings =
      try Env.filter_settings (value "settings") with
          _ ->
            fprintf stderr "\nWARNING: No 'settings' attribute specified for A/C '%s', using 'settings/dummy.xml'\n\n%!" aircraft;
            "settings/dummy.xml" in
    (* add modules settings *)
    let settings = String.concat " " [settings; (try Env.filter_settings (value "settings_modules") with _ -> "")] in

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

    let modules_files = extract_makefile (value "ac_id") abs_airframe_file temp_makefile_ac in

    (* Create Makefile.ac only if needed *)
    let makefile_ac = aircraft_dir // "Makefile.ac" in
    if is_older makefile_ac (abs_airframe_file :: modules_files) then begin
      assert(Sys.command (sprintf "mv %s %s" temp_makefile_ac makefile_ac) = 0)
    end;

    (* Get TARGET env, needed to build modules.h according to the target *)
    let t = try Printf.sprintf "TARGET=%s" (Sys.getenv "TARGET") with _ -> "" in
    make "all_ac_h" t;
    make_opt "radio_ac_h" "RADIO" "radio";
    make_opt "flight_plan_ac_h" "FLIGHT_PLAN" "flight_plan"
  with
      Failure f ->
        prerr_endline f;
        exit 1
