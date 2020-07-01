(*
 * Copyright (C) 2003-2009 Pascal Brisset, Antoine Drouin, ENAC
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * Main generator tool
 *)


open Printf
module U = Unix

module GC = Gen_common
module Af = Airframe
module AfT = Airframe.Target
module AfF = Airframe.Firmware
module GM = Gen_makefile

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let default_conf_xml = paparazzi_conf // "conf.xml"

let airframe_h = "airframe.h"
let flight_plan_h = "flight_plan.h"
let flight_plan_xml = "flight_plan.xml"
let radio_h = "radio.h"
let periodic_h = "periodic_telemetry.h"
let modules_h = "modules.h"
let settings_h = "settings.h"
let settings_xml = "settings.xml"
let default_periodic_freq = 60
let default_modules_freq = 60

let get_string_opt = fun x -> match x with Some s -> s | None -> ""

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


let is_older = fun target_file dep_files ->
  not (Sys.file_exists target_file) ||
    let target_file_time = (U.stat target_file).U.st_mtime in
    let rec loop = function
      | [] -> false
      | f :: fs -> target_file_time < (U.stat f).U.st_mtime || loop fs in
    loop dep_files

let make_element = fun t a c -> Xml.Element (t,a,c)

let copy_file = fun source dest ->
  assert (Sys.command (sprintf "cp %s %s" source dest) = 0)

(** Generate a configuration element
 *  Also check dependencies
 *
 * [a' -> (a' -> unit) -> (string * string list) list -> unit]
 *)
let generate_config_element = fun ?(verbose=true) elt f dep_list ->
  if List.exists (fun (file, dep) -> is_older file dep) dep_list
  then begin if verbose then Printf.printf "(updated)"; f elt end
  else if verbose then Printf.printf "(unchanged)"

(******************************* MAIN ****************************************)
let () =
  let ac_name = ref None
  and target_name = ref None
  and conf_xml = ref default_conf_xml
  and gen_conf = ref false
  and gen_af = ref false
  and gen_fp = ref false
  and gen_set = ref false
  and gen_rc = ref false
  and gen_tl = ref false
  and gen_ap = ref false
  and gen_all = ref false
  and gen_ac_dir = ref None
  and gen_conf_dir = ref None
  and modules_freq = ref default_modules_freq
  and tl_freq = ref default_periodic_freq in

  let options =
    [ "-name", Arg.String (fun x -> ac_name := Some x), " Aircraft name (mandatory)";
      "-target", Arg.String (fun x -> target_name := Some x), " Target to build (mandatory)";
      "-conf", Arg.String (fun x -> conf_xml := x), (sprintf " Configuration file (default '%s')" default_conf_xml);
      "-ac_conf", Arg.Set gen_conf, " Generate expanded aircraft config file";
      "-airframe", Arg.Set gen_af, " Generate airframe file";
      "-flight_plan", Arg.Set gen_fp, " Generate flight plan file";
      "-settings", Arg.Set gen_set, " Generate settings file";
      "-radio", Arg.Set gen_rc, " Generate radio file";
      "-telemetry", Arg.Set gen_tl, " Generate telemetry file";
      "-autopilot", Arg.Set gen_ap, " Generate autopilot file if needed";
      "-all", Arg.Set gen_all, " Generate all files";
      "-ac_dir", Arg.String (fun x -> gen_ac_dir := Some x), "Directory for aircraft generated headers";
      "-conf_dir", Arg.String (fun x -> gen_conf_dir := Some x), "Directory for aircraft generated config";
      "-periodic_freq", Arg.Int (fun x -> tl_freq := x), (sprintf " Periodic telemetry frequency (default %d)" default_periodic_freq);
      "-modules_freq", Arg.Int (fun x -> modules_freq := x), (sprintf " Modules frequency (default %d)" default_modules_freq);
      ] in

  Arg.parse
    options
    (fun x -> Printf.fprintf stderr "%s: Warning: Don't do anything with '%s' argument\n" Sys.argv.(0) x)
    "Usage: ";

  (* check aircraft and target options *)
  let aircraft =
    match !ac_name with
    | None -> failwith "An aircraft name is mandatory"
    | Some ac -> ac
  in
  let target =
    match !target_name with
    | None -> failwith "A target name is mandatory"
    | Some t -> t
  in
  Printf.printf "Aircraft generator: '%s' for target '%s'\n%!" aircraft target;

  try
    let conf = ExtXml.parse_file !conf_xml in
    check_unique_id_and_name conf !conf_xml;
    let aircraft_xml =
      try
        ExtXml.child conf ~select:(fun x -> Xml.attrib x "name" = aircraft) "aircraft"
      with
        Not_found -> failwith (sprintf "Aircraft '%s' not found in '%s'" aircraft !conf_xml)
    in

    let value = fun attrib -> ExtXml.attrib aircraft_xml attrib in

    (* Prepare building folders *)
    let default_aircraft_dir = Env.paparazzi_home // "var" // "aircrafts" // aircraft in
    let aircraft_dir, aircraft_conf_dir = match !gen_conf_dir with
      | None -> default_aircraft_dir, default_aircraft_dir // "conf"
      | Some d -> d, d // "conf"
    in
    let aircraft_gen_dir = match !gen_ac_dir with
      | None -> default_aircraft_dir // target // "generated"
      | Some d -> d // "generated"
    in
    mkdir aircraft_conf_dir;
    mkdir aircraft_gen_dir;


    (*
     *  Parse file if needed
     *)

    let ac = Aircraft.parse_aircraft ~parse_af:!gen_af ~parse_ap:!gen_ap ~parse_fp:!gen_fp ~parse_rc:!gen_rc ~parse_tl:!gen_tl ~parse_set:!gen_set ~parse_all:!gen_all ~verbose:true target aircraft_xml in

    (*
     *  Expands the configuration of the A/C into one single file
     *)

    let configuration =
      make_element
        "configuration" []
        [ make_element "conf" [] ac.Aircraft.xml; PprzLink.messages_xml () ] in
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
      let filename = sprintf "%02d_%02d_%02d__%02d_%02d_%02d_%s_%s.conf"
          (d.U.tm_year mod 100) (d.U.tm_mon+1) (d.U.tm_mday)
          (d.U.tm_hour) (d.U.tm_min) (d.U.tm_sec)
          md5sum aircraft in
      let d = Env.paparazzi_home // "var" // "conf" in
      mkdir d;
      let f = open_out (d // filename) in
      Printf.fprintf f "%s\n" (ExtXml.to_string_fmt configuration);
      close_out f;
      Ocaml_tools.compress (d // filename)
    end;



    (*
     *  Generate output files
     *)


    Printf.printf "Dumping flight plan XML and header...%!";
    let abs_flight_plan_h = aircraft_gen_dir // flight_plan_h in
    let abs_flight_plan_dump = aircraft_dir // flight_plan_xml in
    begin match ac.Aircraft.flight_plan with
      | None -> Printf.printf "(skip)"
      | Some flight_plan ->
        generate_config_element flight_plan
          (fun e ->
             Gen_flight_plan.generate
               e flight_plan.Flight_plan.filename abs_flight_plan_h)
          [ (abs_flight_plan_h, [flight_plan.Flight_plan.filename]) ];
        generate_config_element ~verbose:false flight_plan
          (fun e ->
             Gen_flight_plan.generate
               e ~dump:true flight_plan.Flight_plan.filename abs_flight_plan_dump)
          [ (abs_flight_plan_dump, [flight_plan.Flight_plan.filename]) ];
          (* save conf file in aircraft conf dir *)
        begin match Aircraft.get_element_relative_path (!gen_fp || !gen_all) aircraft_xml "flight_plan" with
          | None -> ()
          | Some f ->
              let dir = (aircraft_conf_dir // (Filename.dirname f)) in
              mkdir dir;
              copy_file flight_plan.Flight_plan.filename dir;
        end;
    end;
    Printf.printf " done\n%!";

    Printf.printf "Dumping airframe header...%!";
    let abs_airframe_h = aircraft_gen_dir // airframe_h in
    begin match ac.Aircraft.airframe with
      | None -> Printf.printf "(skip)"
      | Some airframe ->
        let inc = List.map (fun i ->
          let href = i.Airframe.Include.href in
          Str.global_replace (Str.regexp "\\$AC_ID") (value "ac_id") href
        ) airframe.Af.includes in
        generate_config_element airframe
          (fun e ->
             Gen_airframe.generate
               e (value "ac_id") (get_string_opt !ac_name)
               md5sum airframe.Airframe.filename abs_airframe_h)
          [ (abs_airframe_h, [airframe.Airframe.filename] @ inc) ];
        (* save conf file in aircraft conf dir *)
        begin match Aircraft.get_element_relative_path (!gen_af || !gen_all) aircraft_xml "airframe" with
          | None -> ()
          | Some f ->
              let dir = (aircraft_conf_dir // (Filename.dirname f)) in
              mkdir dir;
              copy_file airframe.Airframe.filename dir;
              copy_file (paparazzi_conf  // "airframes" // "airframe.dtd") (aircraft_conf_dir // "airframes")
        end;
    end;
    Printf.printf " done\n%!";

    Printf.printf "Dumping autopilot header...%!";
    begin match ac.Aircraft.autopilots with
      | None ->  Printf.printf "(skip)"
      | Some autopilots ->
          List.iter (fun (freq, autopilot) ->
            let dep = List.map (fun sm ->
              let h_name = paparazzi_conf // ("autopilot_core_"^(Xml.attrib sm "name")^".h") in
              (h_name, [autopilot.Autopilot.filename])
            ) (Xml.children autopilot.Autopilot.xml) in
            generate_config_element autopilot
            (fun e -> Gen_autopilot.generate e freq aircraft_gen_dir) dep
          ) autopilots
    end;
    Printf.printf " done\n%!";

    Printf.printf "Dumping radio header...%!";
    let abs_radio_h = aircraft_gen_dir // radio_h in
    begin match ac.Aircraft.radio with
      | None -> Printf.printf "(skip)"
      | Some radio ->
        generate_config_element radio
          (fun e -> Gen_radio.generate e radio.Radio.filename abs_radio_h)
          [ (abs_radio_h, [radio.Radio.filename]) ] end;
    Printf.printf " done\n%!";

    Printf.printf "Dumping telemetry header...%!";
    let abs_telemetry_h = aircraft_gen_dir // periodic_h in
    begin match ac.Aircraft.telemetry with
      | None -> Printf.printf "(skip)"
      | Some telemetry ->
        generate_config_element telemetry
          (fun e -> Gen_periodic.generate e !tl_freq abs_telemetry_h)
          [ (abs_telemetry_h, [telemetry.Telemetry.filename]) ] end;
    Printf.printf " done\n%!";

    Printf.printf "Dumping modules header...%!";
    let loaded_modules = Aircraft.get_loaded_modules ac.Aircraft.config_by_target target in
    let abs_modules_h = aircraft_gen_dir // modules_h in
    generate_config_element loaded_modules
      (fun e -> Gen_modules.generate e !modules_freq "" abs_modules_h)
      [ abs_modules_h, List.map (fun m -> m.Module.xml_filename) loaded_modules ];
    Printf.printf " done\n%!";
    

    Printf.printf "Dumping settings XML and header...%!";
    let abs_settings_h = aircraft_gen_dir // settings_h in
    let abs_settings_xml = aircraft_dir // settings_xml in
    begin match ac.Aircraft.settings with
      | None -> Printf.printf "(skip)"
      | Some settings ->
        let dep_list = List.fold_left
          (fun l s -> if Sys.file_exists s.Settings.filename then s.Settings.filename :: l else l) [] settings
        in
        generate_config_element settings
          (fun e -> Gen_settings.generate e dep_list abs_settings_xml abs_settings_h)
          [ (abs_settings_h, dep_list) ] end;
    Printf.printf " done\n%!";


    let temp_makefile_ac = Filename.temp_file "Makefile.ac" "tmp" in
    begin match ac.Aircraft.airframe, ac.Aircraft.flight_plan with
    | Some af, Some fp ->
      GM.generate_makefile (value "ac_id") ac.Aircraft.config_by_target temp_makefile_ac
    | _ -> Printf.eprintf "Missing airframe or flight_plan" end;

    (* Create Makefile.ac only if needed *)
    let makefile_ac = aircraft_dir // "Makefile.ac" in
    match ac.Aircraft.airframe with
    | None -> ()
    | Some airframe ->
      let module_files = List.map (fun m -> m.Module.xml_filename) loaded_modules in
      if is_older makefile_ac (airframe.Airframe.filename :: module_files)
      then
        assert(Sys.command (sprintf "mv %s %s" temp_makefile_ac makefile_ac) = 0)
      ;

  with Failure f ->
    prerr_endline f;
    exit 1
