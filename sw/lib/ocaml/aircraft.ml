(*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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
 * Aircraft module for parsing XML config files
 * Extract the global configuration of an aircraft
 *)
module Af = Airframe
module AfT = Airframe.Target
module AfF = Airframe.Firmware
module GC = Gen_common

let (//) = Filename.concat
let get_string_opt = fun x -> match x with Some s -> s | None -> ""

(* type of loading (user, auto) *)
type load_type = UserLoad | Unloaded | Depend | Suggested

(* configuration sorted by target *)
type target_conf = {
  configures: Module.configure list; (* configure variables *)
  configures_default: Module.configure list; (* default configure options *)
  defines: Module.define list; (* define flags *)
  firmware_name: string;
  board_type: string;
  modules: (load_type * Module.t) list; (* list of modules *)
  autopilot: bool; (* autopilot if any *)
}

(* init target conf structure *)
let init_target_conf = fun firmware_name board_type ->
  { configures = []; configures_default = []; defines = [];
    firmware_name; board_type; modules = []; autopilot = false }

(* global aircraft structure *)
type t = {
  name: string;
  config_by_target: (string, target_conf) Hashtbl.t; (* configuration sorted by target name: (string, target_conf) *)
  all_modules: Module.t list; (* list of all modules *)
  airframe: Airframe.t option;
  autopilots: (string option * Autopilot.t) list option;
  flight_plan: Flight_plan.t option;
  radio: Radio.t option;
  telemetry: Telemetry.t option;
  settings: Settings.t list option;
  xml: Xml.xml list (* config as a list of Xml node *)
}

let init_aircraft_conf = fun name ->
  { name; config_by_target = Hashtbl.create 5; all_modules = [];
    airframe = None; autopilots = None; flight_plan = None;
    radio = None; telemetry = None; settings = None;
    xml = []
  }

(* add a module with configuration *)
let target_conf_add_module_config = fun conf target firmware m load_type ->
  { conf with
    configures = List.fold_left (fun cm mk ->
        if Module.check_mk target firmware mk then
          List.fold_left (fun cmk c ->
            if not (c.Module.cvalue = None) then cmk @ [c]
              else cmk
            ) cm mk.Module.configures
        else
          cm) conf.configures m.Module.makefiles;
    configures_default = List.fold_left (fun cm mk ->
        if Module.check_mk target firmware mk then
          List.fold_left (fun cmk c ->
            if not (c.Module.default = None && c.Module.case = None) then cmk @ [c]
              else cmk
            ) cm mk.Module.configures
        else
          cm) conf.configures_default m.Module.makefiles;
    modules = conf.modules @ [(load_type, m)] }

(* add a module if compatible with target and firmware *)
let target_conf_add_module = fun conf target firmware name mtype load_type ->
  let m = Module.from_module_name mtype name in
  (* check compatibility with target *)
  if Module.check_loading target firmware m then
    (* check if the module itself is already loaded, merging options in all case *)
    let add_module = if List.exists (fun (_, lm) -> m.Module.name = lm.Module.name) conf.modules
      then [] else [(load_type, m)] in
    (* add module but not yet configurations *)
    { conf with modules = conf.modules @ add_module }
  else begin
    (* add "unloaded" module for reference *)
    { conf with modules = conf.modules @ [(Unloaded, m)] } end

type sort_result = {
  resolved: (string * (load_type * Module.t)) list; (* modules to load (fully resolved) *)
  unresolved: (string * Module.t) list;             (* modules no fully resolved (for cycle detection) *)
  unloaded: (load_type * Module.t) list;            (* modules not loaded for this target / firmware *)
  conflicts: (string * string) list;                (* modules in conflict *)
  required: (GC.bool_expr * string) list;           (* functionalities required *)
  provided: (string * string) list;                 (* functionalities provided (should contain required) (func * module name) *)
  suggested: (string, Module.t) Hashtbl.t;          (* modules suggested for missing functionalities (hashtbl key: func) *)
}

let init_sort_result = fun () ->
  { resolved = []; unresolved = []; unloaded = [];
    conflicts = []; required = []; provided = [];
    suggested = Hashtbl.create 5
  }

(* topological sort to load modules and their dependencies *)
let resolve_modules_dep = fun config_by_target firmware user_target ->
  (* recursive dependency resolution *)
  let rec dep_resolve = fun sol m target load_type ->
    let test_module = fun s _m lt ->
      (* test if module is not in resolved *)
      if not (List.mem_assoc _m.Module.name s.resolved) then begin
        (* test if module is in unresolved to detect cycles *)
        if List.mem_assoc _m.Module.name s.unresolved then
          failwith ("Error [Aircraft]: cyclic dependency found when loading module "^_m.Module.name);
        (* no cycle, call resolve function recursively *)
        dep_resolve s _m target lt (* returns sol *)
      end else s
    in
    let name = m.Module.name in
    (* mark module has unresolved *)
    let sol = { sol with unresolved = sol.unresolved @ [(name, m)] } in
    (* if module should be loaded, look for dependencies *)
    let sol =
      if Module.check_loading target firmware m then begin
        match m.Module.dependencies with
        | Some dep ->
            (* iter over requires *)
            let sol = List.fold_left (fun s dep_expr ->
              match dep_expr with
              | GC.Var dep_name ->
                  if Str.string_match (Str.regexp "^@.*") dep_name 0 then
                    (* add to required list *)
                    { s with required = s.required @ [(dep_expr, name)] }
                  else
                    (* get module from name *)
                    let _m = Module.from_module_name None dep_name in
                    test_module s _m (if name = "root" then UserLoad else Depend)
              | _ -> { s with required = s.required @ [(dep_expr, name)] } (* expression of required modules or functionalities *)
            ) sol dep.Module.requires in
            (* add suggests from root as normal modules with correct load type *)
            let sol =
              if name = "root" then
                List.fold_left (fun s suggested -> test_module s (Module.from_module_name None suggested) Suggested) sol dep.Module.suggests
              else sol
            in
            (* add conflicts to list *)
            let sol = { sol with conflicts = sol.conflicts @ (List.map (fun c -> (c, name)) dep.Module.conflicts) } in
            (* add provides to list *)
            let sol = { sol with provided = sol.provided @ List.map (fun p -> (p, name)) dep.Module.provides } in
            (* add suggests to list *)
            List.iter (fun s ->
              let suggested_module = Module.from_module_name None s in
              match suggested_module.Module.dependencies with
              | Some dep_suggested -> List.iter (fun p -> Hashtbl.add sol.suggested p suggested_module) dep_suggested.Module.provides
              | None -> ()
            ) dep.Module.suggests;
            (* all dep resolved, add to list *)
            if not (name = "root") then (* don't add root module *)
              { sol with resolved = sol.resolved @ [(name, (load_type, m))] } (* add to list and return solution *)
            else sol (* return current solution *)
        | None ->
            (* no dep, add to list *)
            if not (name = "root") then (* don't add root module *)
              { sol with resolved = sol.resolved @ [(name, (load_type, m))] } (* add to list and return solution *)
            else sol (* return current solution *)
      end else begin
        (* not adding module, but still add suggested *)
        let _ = match m.Module.dependencies with
          | Some dep ->
              List.iter (fun s ->
                let suggested_module = Module.from_module_name None s in
                match suggested_module.Module.dependencies with
                | Some dep_suggested -> List.iter (fun p -> Hashtbl.add sol.suggested p suggested_module) dep_suggested.Module.provides
                | None -> ()
              ) dep.Module.suggests
          | None -> ()
        in
        (* return resolved but not adding module *)
        { sol with unloaded = sol.unloaded @ [(Unloaded, m)] }
      end
    in
    (* remove from unresolved list to make search faster and return current solution *)
    { sol with unresolved = List.remove_assoc name sol.unresolved }
  in
  (* ordering solution after finding all dependencies *)
  let dep_order = fun sol ->
    let dep_list = List.map (fun (n, (_, m)) ->
      let rec extract_dep l = function
        | GC.Any | GC.Not _ -> l
        | GC.And (e1, e2) | GC.Or (e1, e2) -> extract_dep (extract_dep l e1) e2
        | GC.Var s ->
            if Str.string_match (Str.regexp "^@.*") s 0 then
              (* substitute functionality with module names *)
              List.fold_left (fun acc (f, m_name) -> if String.compare f s = 0 then m_name :: acc else acc) l sol.provided
            else
              s :: l (* return normal module name *)
      in
      let depend_names = match m.Module.dependencies with
        | Some d -> GC.singletonize (List.fold_left extract_dep [] (d.Module.requires @ d.Module.recommends))
        | None -> []
      in
      n, depend_names
      ) sol.resolved
    in
    (* list items, each being unique
     * code from https://rosettacode.org/wiki/Topological_sort#OCaml
    *)
    let all = GC.singletonize (List.flatten (List.map (fun (m, dep) -> m::dep) dep_list)) in
    (* second topologic sort function *)
    let get_deps = fun x -> try List.assoc x dep_list with Not_found -> [] in
    let rec order acc later todo progress =
      match todo, later with
      | [], [] -> (List.rev acc)
      | [], _ ->
          if progress then order acc [] later false
          else begin
            let fail_s = List.fold_left (fun s m -> Printf.sprintf "%s '%s'" s m) "" later in
            failwith (Printf.sprintf "Error [Aircraft]: un-orderable modules %s" fail_s)
          end
      | x::xs, _ ->
          let deps = get_deps x in
          let ok = List.for_all (fun dep -> List.mem dep acc) deps in
          if ok then order (x::acc) later xs true
          else order acc (x::later) xs progress
    in
    let starts, todo = List.partition (fun x -> get_deps x = []) all in
    let res = order starts [] todo false in
    (* apply new order to solution *)
    let resolved = List.fold_left (fun l m -> try (m, List.assoc m sol.resolved) :: l with Not_found -> l) [] res in
    { sol with resolved=List.rev resolved }
  in
  (* iter on all targets *)
  Hashtbl.iter (fun target conf ->
    (* check if a board, target or firmware specific module is available *)
    let target_module_name = Env.paparazzi_conf // "modules" // "targets" // target ^ ".xml" in
    let target_module = if Sys.file_exists target_module_name then [GC.Var target_module_name] else [] in
    let firmware_module_name = Env.paparazzi_conf // "modules" // "firmwares" // conf.firmware_name ^ ".xml" in
    let firmware_module = if Sys.file_exists firmware_module_name then [GC.Var firmware_module_name] else [] in
    let board_module_name = Env.paparazzi_conf // "modules" // "boards" // conf.board_type ^ ".xml" in
    let board_module = if Sys.file_exists board_module_name then [GC.Var board_module_name] else [] in
    (* iter on modules of this target from a meta module *)
    let root_dep = { Module.empty_dep with Module.requires = (List.map (fun (_, m) -> GC.Var m.Module.name) conf.modules) @ target_module @ firmware_module @ board_module } in
    let root_module = {
      Module.empty with
      Module.name = "root";
      Module.dependencies = Some root_dep;
      Module.makefiles = [Module.empty_makefile]
    } in
    let solution = dep_resolve (init_sort_result ()) root_module target UserLoad in
    (* test for conflicts functionalities and option if requested *)
    if (not (user_target = "")) && (target = user_target) then begin
      (* check conflicts for resolved modules *)
      List.iter (fun (c, cname) ->
        List.iter (fun (name, _) ->
          if name = c then
            failwith (Printf.sprintf "Error [Aircraft]: find conflict with module '%s' while loading '%s' in target '%s'" cname name target)
        ) solution.resolved;
        List.iter (fun (name, _) ->
          if name = c then
            failwith (Printf.sprintf "Error [Aircraft]: find conflict with funcionality while loading '%s' for '%s' in target '%s'" name cname target)
        ) solution.provided
      ) solution.conflicts;
    end;
    (* chek that all required functionalities or modules are provided
     * if not, search suggested list of modules, fail if nothing found
     *)
    let modules_and_func = (fst (List.split solution.provided)) @ (fst (List.split solution.resolved)) in
    let selection = List.fold_left (fun s (r, name) ->
      if (List.exists (fun p -> GC.eval_bool p r) modules_and_func) then s (* functionality is provided *)
      else
        let select = Hashtbl.fold (fun p m l -> if GC.eval_bool p r then m :: l else l) solution.suggested [] in
        if List.length select = 0 && (not (user_target = "")) && (target = user_target) then
          failwith (Printf.sprintf "Error [Aircraft]: functionality '%s' is not provided for '%s' in target '%s'" (GC.sprint_expr r) name target)
        else
          s @ select (* return selection *)
    ) [] solution.required in
    let solution = match selection with
    | [] -> solution (* nothing to change *)
    | _ ->
        (* add selection to root dep *)
        let root_dep = { root_dep with Module.suggests = (List.map (fun m -> m.Module.name) selection) } in
        let root_module = { root_module with Module.dependencies = Some root_dep } in
        dep_resolve (init_sort_result ()) root_module target UserLoad
    in
    (* find final order *)
    let solution = dep_order solution in
    (* add configure, defines and modules to conf for all resolved modules *)
    let new_conf = List.fold_left (fun c (lt, m) ->
      target_conf_add_module_config c target firmware m lt
    ) { conf with modules = solution.unloaded } (snd (List.split solution.resolved)) in
    (*let new_conf = { new_conf with modules = new_conf.modules @ !unloaded } in*)
    Hashtbl.replace config_by_target target new_conf
  ) config_by_target


(* sort element of an airframe type by target *)
let sort_airframe_by_target = fun config_by_target airframe ->
  match airframe with
  | None -> []
  | Some a ->
    (* build a list of pairs (target, firmware) *)
    let l = List.fold_left (fun lf f ->
        List.fold_left (fun lt t ->
            lt @ [(t, f)]) lf f.AfF.targets
      ) [] a.Af.firmwares in
    (* iterate on each target *)
    List.iter (fun (t, f) ->
        let name = t.AfT.name in (* target name *)
        if Hashtbl.mem config_by_target name then
          failwith "Error [Aircraft]: two targets with the same name";
        (* init and add configure/define from airframe *)
        let conf = init_target_conf f.AfF.name t.AfT.board in
        let conf = { conf with
                     configures = t.AfT.configures @ f.AfF.configures;
                     defines = t.AfT.defines @ f.AfF.defines } in
        (* iter on modules in target *)
        let conf = List.fold_left (fun c m_af ->
            let c = { c with
                      configures = c.configures @ m_af.Module.configures;
                      defines = c.defines @ m_af.Module.defines } in
            target_conf_add_module c name f.AfF.name m_af.Module.name m_af.Module.mtype UserLoad
          ) conf t.AfT.modules in
        (* iter on modules in firmwares *)
        let conf = List.fold_left (fun c m_af ->
            let c = { c with
                      configures = c.configures @ m_af.Module.configures;
                      defines = c.defines @ m_af.Module.defines } in
            target_conf_add_module c name f.AfF.name m_af.Module.name m_af.Module.mtype UserLoad
          ) conf f.AfF.modules in
        (* iter on deprecated 'modules' node *)
        let conf = List.fold_left (fun c m ->
          List.fold_left (fun c m_af ->
            let c = { c with
                      configures = c.configures @ m_af.Module.configures;
                      defines = c.defines @ m_af.Module.defines } in
            target_conf_add_module c name f.AfF.name m_af.Module.name m_af.Module.mtype UserLoad
            ) c m.Airframe.OldModules.modules
          ) conf a.Airframe.modules in
        Hashtbl.add config_by_target name conf
    ) l;
    (* return list of targets * firmwares *)
    l

(** Extract a configuration element from aircraft config,
 *  returns a tuple with absolute file path and element object
 * 
 * [bool -> Xml.xml -> string -> (Xml.xml -> a') -> (string * a' option)]
 *)
let get_config_element = fun flag ac_xml elt f ->
  if not flag then None
  else
    try
      let file = Xml.attrib ac_xml elt in
      let abs_file = Env.paparazzi_conf // file in
      Some (f abs_file)
    with Xml.No_attribute _ -> None (* no attribute elt in conf file *)

let get_element_relative_path = fun flag ac_xml elt ->
  if not flag then None
  else
    try
      Some (Xml.attrib ac_xml elt)
    with Xml.No_attribute _ -> None (* no attribute elt in conf file *)


(** Extract loaded modules from hashtbl config
 *)
let get_loaded_modules = fun config_by_target target ->
  try
    let config = Hashtbl.find config_by_target target in
    let modules = config.modules in
    (List.fold_left (fun
      (lt, lm) (t, m) ->
        if t <> Unloaded then (lt @ [t], lm @ [m]) else (lt, lm)) ([], []) modules)
  with Not_found -> ([], []) (* nothing for this target *)

(** Extract all modules
 *  if a modules is not in any target, it will not appear in the list
 *  returns an alphabetically sorted list
 *)
let get_all_modules = fun config_by_target ->
  let modules = ref [] in
  Hashtbl.iter (fun _ conf ->
    List.iter (fun (_, m) ->
      if not (List.exists (fun n -> m.Module.name = n.Module.name) !modules) then
        modules := m :: !modules (* add module to list *)
    ) conf.modules
  ) config_by_target;
  List.sort (fun m1 m2 -> compare m1.Module.name m2.Module.name) !modules


let parse_aircraft = fun ?(parse_af=false) ?(parse_ap=false) ?(parse_fp=false) ?(parse_rc=false) ?(parse_tl=false) ?(parse_set=false) ?(parse_all=false) ?(verbose=false) target aircraft_xml ->

  let name = ExtXml.attrib aircraft_xml "name" in
  let conf_aircraft = [] in (* accumulate aircraft XML config *)
  let config_by_target = Hashtbl.create 5 in


  if verbose then
    Printf.printf "Parsing airframe%!";
  let airframe = get_config_element (parse_af || parse_all) aircraft_xml "airframe" Airframe.from_file in
  if verbose then
    Printf.printf " '%s'%!" (match airframe with None -> "None" | Some a -> a.Airframe.filename);
  let conf_aircraft = conf_aircraft @ (match airframe with None -> [] | Some x -> [x.Airframe.xml]) in
  if verbose then
    Printf.printf ", sorting by target%!";
  let target_list = sort_airframe_by_target config_by_target airframe in
  let firmware = try let (_, f) = List.find (fun (t, f) -> t.AfT.name = target) target_list in f.AfF.name with Not_found -> "none" in
  if verbose then
    Printf.printf ", extracting and parsing autopilot...%!";
  let autopilots = if parse_ap || parse_all then
    begin
      match airframe with
      | None -> None
      | Some af ->
          (* extract autopilots *)
          let autopilots = List.fold_left (fun lf f ->
            let ap_f = match f.Airframe.Firmware.autopilot with None -> [] | Some a -> [a] in
            let ap_t = List.fold_left (fun lt t ->
              if t.Airframe.Target.name = target then
                match t.Airframe.Target.autopilot with None -> lt | Some a -> a :: lt
              else lt
            ) ap_f f.Airframe.Firmware.targets in
            ap_t @ lf
          ) af.Airframe.autopilots af.Airframe.firmwares in
          if List.length autopilots = 0 then None
          else
            let autopilots = List.map (fun af_ap ->
              let filename = af_ap.Airframe.Autopilot.name in
              let filename = if Filename.check_suffix filename ".xml"
                             then filename
                             else filename^".xml" in
              let filename = Env.paparazzi_conf // "autopilot" // filename in
              let ap = Autopilot.from_file filename in
              (* extract modules from autopilot *)
              Hashtbl.iter (fun target conf ->
                let conf = List.fold_left (fun c m ->
                    let c = { c with
                              configures = c.configures @ m.Module.configures;
                              defines = c.defines @ m.Module.defines } in
                    target_conf_add_module c target firmware m.Module.name m.Module.mtype UserLoad
                ) conf ap.Autopilot.modules in
                Hashtbl.replace config_by_target target conf
              ) config_by_target;
              (af_ap.Airframe.Autopilot.freq, ap)
            ) autopilots in
            try
              let c = Hashtbl.find config_by_target target in
              Hashtbl.replace config_by_target target { c with autopilot = true };
              Some autopilots
            with Not_found ->
              (* target not found or not defined, add in all targets instead *)
              List.iter (fun (t, _) ->
                let c = Hashtbl.find config_by_target t.AfT.name in
                Hashtbl.replace config_by_target target { c with autopilot = true }
              ) target_list;
              Some autopilots
    end
  else None in
  let conf_aircraft = conf_aircraft @ (match autopilots with None -> [] | Some lx -> List.map (fun (_, x) -> x.Autopilot.xml) lx) in
  if verbose then
    Printf.printf " done.\n%!";
  
  if verbose then
    Printf.printf "Parsing flight plan%!";
  let flight_plan = get_config_element (parse_fp || parse_all) aircraft_xml "flight_plan" Flight_plan.from_file in
  if verbose then begin
    Printf.printf " '%s'%!" (match flight_plan with None -> "None" | Some fp -> fp.Flight_plan.filename);
    Printf.printf ", extracting modules...%!"
  end;
  begin match flight_plan with
    | None -> ()
    | Some fp ->
      Hashtbl.iter (fun target conf ->
        let conf = List.fold_left (fun c m ->
            let c = { c with
                      configures = c.configures @ m.Module.configures;
                      defines = c.defines @ m.Module.defines } in
            target_conf_add_module c target firmware m.Module.name m.Module.mtype UserLoad
        ) conf fp.Flight_plan.modules in
        Hashtbl.replace config_by_target target conf
      ) config_by_target
  end;
  let conf_aircraft = conf_aircraft @ (match flight_plan with None -> [] | Some x -> [x.Flight_plan.xml]) in
  if verbose then
    Printf.printf " done\n%!";

  if verbose then
    Printf.printf "Parsing radio%!";
  let radio = get_config_element (parse_rc || parse_all) aircraft_xml "radio" Radio.from_file in
  if verbose then
    Printf.printf " '%s'...%!" (match radio with None -> "None" | Some rc -> rc.Radio.filename);
  let conf_aircraft = conf_aircraft @ (match radio with None -> [] | Some x -> [x.Radio.xml]) in
  if verbose then
    Printf.printf " done\n%!";

  if verbose then
    Printf.printf "Parsing telemetry%!";
  let telemetry = get_config_element (parse_tl || parse_all) aircraft_xml "telemetry" Telemetry.from_file in
  if verbose then
    Printf.printf " '%s'...%!" (match telemetry with None -> "None" | Some tl -> tl.Telemetry.filename);
  let conf_aircraft = conf_aircraft @ (match telemetry with None -> [] | Some x -> [x.Telemetry.xml]) in
  if verbose then
    Printf.printf " done\n%!";

  (* resolve modules dep *)
  (* don't fail if no target specified, execpt for cyclic dependencies *)
  resolve_modules_dep config_by_target firmware target;
  let loaded_types, loaded_modules = get_loaded_modules config_by_target target in
  let all_modules = get_all_modules config_by_target in

  if verbose then
    Printf.printf "Parsing settings...%!";
  let settings =
    if parse_set || parse_all then begin
      (* normal settings *)
      let settings = try Env.filter_settings (ExtXml.attrib aircraft_xml "settings") with _ -> "" in
      let settings_files = Str.split (Str.regexp " ") settings in
      let settings = List.map
        (fun f -> Settings.from_file (Env.paparazzi_conf // f)) settings_files in
      (* modules settings *)
      let settings_modules =
        try Env.filter_settings (ExtXml.attrib aircraft_xml "settings_modules")
        with _ -> "" in
      let settings_modules_files = Str.split (Str.regexp " ") settings_modules in
      let settings_modules = List.fold_left
          (fun acc m ->
            if List.exists (fun name ->
              let name = List.hd (Str.split (Str.regexp "~") name) in
              (* FIXME use correctly the settings name in filtered settings ? *)
              m.Module.xml_filename = (if Filename.is_relative name
              then (Env.paparazzi_conf // name) else name)) settings_modules_files
            then acc @ m.Module.settings else acc
          ) [] loaded_modules in
      (* system settings *)
      let sys_tl_settings = Telemetry.get_sys_telemetry_settings telemetry in
      let sys_mod_settings = Module.get_sys_modules_settings loaded_modules in
      let sys_fp_settings = Flight_plan.get_sys_fp_settings flight_plan in
      let sys_ap_settings = Autopilot.get_sys_ap_settings autopilots in
      (* filter system settings *)
      let system_settings = List.fold_left
        (fun l s -> match s with None -> l | Some x -> x::l) []
        [sys_tl_settings; sys_fp_settings; sys_mod_settings; sys_ap_settings]
      in
      (* group into a common System dl_settings *)
      let sys_dl_settings = List.fold_left (fun l s -> s.Settings.dl_settings @ l) [] system_settings in
      let sys_dl_settings = {
        Settings.Dl_settings.name = Some "System";
        dl_settings = sys_dl_settings; dl_setting = []; headers = [];
        xml = Xml.Element ("dl_settings", [("name","System")], (List.map (fun s -> s.Settings.Dl_settings.xml) sys_dl_settings))
      } in
      let system_settings = {
        Settings.filename = ""; name = None; target = None;
        dl_settings = [sys_dl_settings];
        xml = Xml.Element ("settings", [], [Xml.Element ("dl_settings", [], (List.map (fun s -> s.Settings.Dl_settings.xml) [sys_dl_settings]))])
      } in
      (* join all settings in correct order *)
      let settings = [system_settings] @ settings @ settings_modules in
      (* filter on targets *)
      let settings = List.fold_left (fun l s ->
        if GC.test_targets target (GC.bool_expr_of_string s.Settings.target) then s :: l
        else l
      ) [] settings in
      Some (List.rev settings)
    end
    else None
  in
  let conf_aircraft = conf_aircraft @ (match settings with None -> [] | Some x -> [Settings.get_settings_xml x]) in
  if verbose then
    Printf.printf " done\n%!";

  if verbose then begin
    let letter_of_load_tyoe = function
      | UserLoad -> "U" | Depend -> "D" | Unloaded -> "N" | Suggested -> "S"
    in
    Printf.printf "Loading modules:\n";
  List.iter2 (fun lt m ->
    Printf.printf " - (%s) %s (%s) [%s]\n" (letter_of_load_tyoe lt) m.Module.name (get_string_opt m.Module.dir) m.Module.xml_filename) loaded_types loaded_modules
  end;

  (* return aircraft conf *)
  { name; config_by_target; all_modules;
    airframe; autopilots; flight_plan; radio; telemetry; settings;
    xml = conf_aircraft }

