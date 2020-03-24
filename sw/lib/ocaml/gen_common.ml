(*
 * generic tools for modules
 *
 * Copyright (C) 2010 Gautier Hattenberger
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

exception Firmware_Found of Xml.xml

(** simple boolean expressions *)
type bool_expr =
  | Var of string
  | Not of bool_expr
  | And of bool_expr * bool_expr
  | Or of bool_expr * bool_expr

(** evaluate a boolean expression for a given value *)
let rec eval_bool v = function
  | Var x -> v = x
  | Not e -> not (eval_bool v e)
  | And (e1, e2) -> eval_bool v e1 && eval_bool v e2
  | Or (e1, e2) -> eval_bool v e1 || eval_bool v e2

(** pretty print boolean expression *)
let print_bool = fun v e ->
  let rec print_b v = function
    | Var x -> eprintf "Var ( %s =? %s ) " x v
    | Not e -> eprintf "Not ( "; (print_b v e); eprintf ") "
    | And (e1, e2) -> eprintf "And ( "; print_b v e1; print_b v e2; eprintf ") "
    | Or (e1, e2) -> eprintf "Or ( "; print_b v e1; print_b v e2; eprintf ") "
  in
  print_b v e; eprintf "\n"


type module_conf = {
    name: string;
    xml: Xml.xml;
    file: string;
    filename: string;
    vpath: string option;(* this field should be removed after transition phase *)
    param: Xml.xml list;
    targets: bool_expr
  }

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let modules_dir = paparazzi_conf // "modules"
let autopilot_dir = paparazzi_conf // "autopilot"

(** remove all duplicated elements of a list *)
let singletonize = fun ?(compare = compare) l ->
  let rec loop = fun l ->
    match l with
    | [] | [_] -> l
    | x::((x'::_) as xs) -> if compare x x' = 0 then loop xs else x::loop xs in
  loop (List.sort compare l)

(** union of two lists *)
let union = fun l1 l2 -> singletonize (l1 @ l2)

(** union of a list of list *)
let union_of_lists = fun l -> singletonize (List.flatten l)

(** [targets_of_field]
 * Returns the targets expression of a makefile node in modules
 * Default "ap|sim" *)
let targets_of_field =
  let rec expr_of_targets op = function
    | [] -> Var ""
    | [e] -> Var e
    | l::ls -> op (Var l) (expr_of_targets op ls)
  in
  let pipe = Str.regexp "|" in
  fun field default ->
    let f = ExtXml.attrib_or_default field "target" default in
    if String.length f > 0 && String.get f 0 = '!' then
      Not (expr_of_targets (fun x y -> Or(x,y)) (Str.split pipe (String.sub f 1 ((String.length f) - 1))))
    else
      expr_of_targets (fun x y -> Or(x,y)) (Str.split pipe f)

(** [get_autopilot_of_airframe xml]
    * Returns (autopilot xml, main freq) from airframe xml file *)
let get_autopilot_of_airframe = fun ?target xml ->
  (* first, find firmware related to the target *)
  let firmware =
    match target with
    | None -> None
    | Some t -> begin try
        Xml.iter (fun x ->
          if Xml.tag x = "firmware" then begin
            Xml.iter (fun xt ->
              if Xml.tag xt = "target" then begin
                if Xml.attrib xt "name" = t then raise (Firmware_Found x)
              end) x
          end) xml;
          None
        with Firmware_Found f -> Some f | _ -> None
    end
  in
  (* extract all autopilot node from xml tree for correct target *)
  let rec iter = fun targets ap xml ->
    match xml with
    | Xml.PCData _ -> ap
    | Xml.Element (tag, _attrs, children) when tag = "autopilot" ->
        [Xml.Element (tag, _attrs, children)] @ ap (* found an autopilot *)
    | Xml.Element (tag, _attrs, children) when tag = "firmware" ->
        begin match firmware with
        | Some f when String.compare (Xml.to_string f) (Xml.to_string xml) = 0 ->
            List.fold_left (fun acc xml ->
              iter targets acc xml) ap children
        | None ->
            List.fold_left (fun acc xml ->
              iter targets acc xml) ap children
        | _ -> ap end (* skip wrong firmware *)
    | Xml.Element (tag, _attrs, children) when tag = "target" ->
        let target_name = Xml.attrib xml "name" in
        begin match target with
        | None ->
            List.fold_left
              (fun acc xml -> iter targets acc xml) ap children
        | Some t when t = target_name ->
            List.fold_left
              (fun acc xml -> iter targets acc xml) ap children
        | _ -> ap end (* skip wrong target *)
    | Xml.Element (tag, _attrs, children) ->
        List.fold_left
          (fun acc xml -> iter targets acc xml) ap children
  in
  let ap = iter target [] xml in
  (* Raise error if more than one modules section *)
  match ap with
      [autopilot] ->
        let freq = try Some (Xml.attrib autopilot "freq") with _ -> None in
        let ap = try Xml.attrib autopilot "name" with _ -> raise Not_found in
        (autopilot_dir // ap, freq)
    | [] -> raise Not_found
    | _ -> failwith "Error: you have more than one 'autopilot' section per firmware/target in your airframe file"

(** [get_targets_of_module xml]
 * Returns the boolean expression of targets of a module *)
let get_targets_of_module = fun xml ->
  Xml.fold (fun a x ->
    match Compat.lowercase_ascii (Xml.tag x) with
    | "makefile" when a = Var "" -> targets_of_field x Env.default_module_targets
    | "makefile" -> Or (a, targets_of_field x Env.default_module_targets)
    | _ -> a
  ) (Var "") xml

let module_name = fun xml ->
  let name = ExtXml.attrib xml "name" in
  try if Filename.check_suffix name ".xml" then Filename.chop_extension name else name with _ -> name

exception Subsystem of string
let get_module = fun m global_targets ->
  match Xml.tag m with
  | "module" | "autoload" ->
      let name = module_name m in
      let filename =
        let modtype = ExtXml.attrib_or_default m "type" "" in
        name ^ (if modtype = "" then "" else "_") ^ modtype ^ ".xml" in
      let file = modules_dir // filename in
      if not (Sys.file_exists file) then raise (Subsystem file) else
      let xml = ExtXml.parse_file file in
      let targets = get_targets_of_module xml in
      let targets = Or (global_targets, targets) in
      { name = name; xml = xml; file = file; filename = filename; vpath = None;
        param = Xml.children m; targets = targets }
  | "load" -> (* this case should be removed after transition phase *)
      let dir, vpath =
        try
          let dir = ExtXml.attrib m "dir" in
          let dir =
            if Filename.is_relative dir then Env.paparazzi_home // dir
            else dir in
          (dir, Some dir)
        with _ -> modules_dir, None in
      let filename = ExtXml.attrib m "name" in
      let name = Filename.chop_extension filename in
      let file = dir // filename in
      let xml = ExtXml.parse_file file in
      let targets = get_targets_of_module xml in
      let extra_targets = Or (global_targets, targets_of_field m "") in
      let targets = Or (extra_targets, targets) in
      { name = name; xml = xml; file = file; filename = filename; vpath = vpath;
        param = Xml.children m; targets = targets }
  | _ -> Xml2h.xml_error "module, autoload or load"

(** [get_autoloaded_modules module]
 * Return a list of modules to be automaticaly added
 * Only works with actual modules (no subsystems) *)
let rec get_autoloaded_modules = fun m ->
  let m = get_module m (Var "") in
  List.fold_left (fun l t ->
    if ExtXml.tag_is t "autoload" then
      let am = get_module t (Var "") in
      (am :: ((try get_autoloaded_modules am.xml with _ -> []) @ l))
    else l
  ) [] (Xml.children m.xml)

(** [test_targets target targets]
 * Test if [target] is allowed [targets]
 * Return true if target is allowed, false if target is not in list or rejected (prefixed by !) *)
let test_targets = fun target targets ->
  eval_bool target targets

(** [expand_includes ac_id xml]
 * Expand xml airframe file if it contains 'include' nodes
 *)
let expand_includes = fun ac_id xml ->
  match xml with
  | Xml.PCData d -> Xml.PCData d
  | Xml.Element (tag, attrs, children) ->
      Xml.Element (tag, attrs,
      List.fold_left (fun x c ->
        if Xml.tag c = "include" then begin
          let filename = Str.global_replace (Str.regexp "\\$AC_ID") ac_id (ExtXml.attrib c "href") in
          let filename =
            if Filename.is_relative filename then Env.paparazzi_home // filename
            else filename in
          let subxml = ExtXml.parse_file filename in
          x @ (Xml.children subxml)
        end
        else x @ [c]
      ) [] children)

(** [get_modules_of_airframe xml]
 * Returns a list of module configuration from airframe file *)
let rec get_modules_of_airframe = fun ?target xml ->
  let is_module = fun tag -> List.mem tag [ "module"; "load" ] in
  (* first, find firmware related to the target *)
  let firmware =
    match target with
    | None -> None
    | Some t -> begin try
        Xml.iter (fun x ->
          if Xml.tag x = "firmware" then begin
            Xml.iter (fun xt ->
              if Xml.tag xt = "target" then begin
                if Xml.attrib xt "name" = t then raise (Firmware_Found x)
              end) x
          end) xml;
          None
        with Firmware_Found f -> Some f | _ -> None
    end
  in
  (* extract modules from xml tree *)
  let rec iter_modules = fun ?(subsystem_fallback=true) targets modules xml ->
    match xml with
    | Xml.PCData _ -> modules
    | Xml.Element (tag, _attrs, children) when is_module tag ->
        begin try
          let m = get_module xml targets in
          let al = get_autoloaded_modules xml in
          List.fold_left
            (fun acc xml -> iter_modules targets acc xml)
            (m :: (al @ modules)) children
        with Subsystem file ->
          if subsystem_fallback then modules
          else failwith ("Unkown module " ^ file)
        end
    | Xml.Element (tag, _attrs, children) when tag = "firmware" ->
        begin match firmware with
        | Some f when String.compare (Xml.to_string f) (Xml.to_string xml) = 0 ->
            List.fold_left (fun acc xml ->
              iter_modules targets acc xml) modules children
        | None ->
            List.fold_left (fun acc xml ->
              iter_modules targets acc xml) modules children
        | _ -> modules end (* skip wrong firmware *)
    | Xml.Element (tag, _attrs, children) when tag = "target" ->
        let target_name = Xml.attrib xml "name" in
        begin match target with
        | None ->
            List.fold_left
              (fun acc xml -> iter_modules targets acc xml) modules children
        | Some t when t = target_name ->
            List.fold_left
              (fun acc xml -> iter_modules targets acc xml) modules children
        | _ -> modules end
    | Xml.Element (tag, _attrs, _children) when tag = "include" ->
        let filename = ExtXml.attrib xml "href" in
        let subxml = ExtXml.parse_file filename in
        iter_modules targets modules subxml
    | Xml.Element (tag, _attrs, children) ->
        let (targets, use_fallback) =
          if tag = "modules" then (targets_of_field xml "", false) else (targets, true) in
        List.fold_left
          (fun acc xml -> iter_modules ~subsystem_fallback:use_fallback targets acc xml) modules children in
  let modules = iter_modules (Var "") [] xml in
  let ap_modules =
    try
      let ap_file = fst (get_autopilot_of_airframe ?target xml) in
      iter_modules (Var "") [] (ExtXml.parse_file ap_file)
    with _ -> [] in
  let modules = List.rev (ap_modules @ modules) in
  match target with
  | None -> modules
  | Some t -> List.filter (fun m -> test_targets t m.targets) modules


(** [get_modules_of_flight_plan xml]
 * Returns a list of module configuration from flight plan file *)
let get_modules_of_flight_plan = fun xml ->
  let rec iter_modules = fun targets modules xml ->
    match xml with
    | Xml.PCData _ -> modules
    | Xml.Element (tag, _attrs, children) when tag = "module" ->
        begin try
          let m = get_module xml targets in
          List.fold_left
            (fun acc xml -> iter_modules targets acc xml)
            (m :: modules) children
        with _ -> modules end
    | Xml.Element (tag, _attrs, children) ->
        List.fold_left
          (fun acc xml -> iter_modules targets acc xml) modules children in
  List.rev (iter_modules (Var "") [] xml)

(** [singletonize_modules xml]
 * Returns a list of singletonized modules were options are merged
 *)
let singletonize_modules = fun ?(verbose=false) ?target xml ->
  let rec loop = fun l ->
    match l with
    | [] | [_] -> l
    | x::xs ->
        let (duplicates, rest) = List.partition (fun m -> m.file = x.file) xs in
        if List.length duplicates > 0 && verbose then begin
          (* print info message on stderr *)
          let t = match target with None -> "" | Some t -> Printf.sprintf " for target %s" t in
          Printf.eprintf "Info: module '%s' has been loaded several times%s, merging options\n" x.filename t;
          List.iter (fun opt ->
            let name = Xml.attrib opt "name" in
            List.iter (fun d ->
              List.iter (fun d_opt ->
                if Xml.attrib d_opt "name" = name then
                  Printf.eprintf "Warning: - option '%s' is defined multiple times, this may cause unwanted behavior or compilation errors\n" name
              ) d.param;
            ) duplicates;
          ) x.param;
        end;
        let m = { name = x.name; xml = x.xml; file = x.file; filename = x.filename;
        vpath = x.vpath; param = List.flatten (List.map (fun m -> m.param) ([x] @ duplicates));
        targets = List.fold_left (fun a x ->
          match a with
          | Var "" -> x.targets
          | _ -> Or (a, x.targets)
        ) (Var "") ([x] @ duplicates) } in
        m::loop rest
  in
  loop xml

(** [get_modules_of_config ?target flight_plan airframe]
 * Returns a list of pair (modules ("load" node), targets) from airframe file and flight plan.
 * The modules are singletonized and options are merged *)
let get_modules_of_config = fun ?target ?verbose ac_id af_xml fp_xml ->
  let af_modules = get_modules_of_airframe ?target (expand_includes ac_id af_xml)
  and fp_modules = get_modules_of_flight_plan fp_xml in
  (* singletonize modules list and reverse list to have it in the correct order *)
  List.rev (singletonize_modules ?verbose ?target (af_modules @ fp_modules))

(** [get_modules_name xml]
 * Returns a list of loaded modules' name *)
let get_modules_name = fun ac_id xml ->
  let target = try Sys.getenv "TARGET" with _ -> "" in
  (* extract all modules sections for a given target *)
  let modules = get_modules_of_airframe ~target (expand_includes ac_id xml) in
  (* return a list of modules name *)
  List.map (fun m -> ExtXml.attrib m.xml "name") modules

(** [get_modules_dir xml]
    * Returns the list of modules directories *)
let get_modules_dir = fun modules ->
  let dir = List.map (fun m -> try Xml.attrib m.xml "dir" with _ -> ExtXml.attrib m.xml "name") modules in
  singletonize dir

(** [is_element_unselected target modules file]
 * Returns True if [target] is supported in the element [file] and, if it is
 * a module, that it is loaded,
 * [file] being the file name of an Xml file (module or setting) *)
let is_element_unselected = fun ?(verbose=false) target modules name ->
  try
    let name = (Env.paparazzi_home // "conf" // name) in
    let xml = ExtXml.parse_file name in
    match Xml.tag xml with
    | "settings" ->
        let target_list = targets_of_field xml "" in
        let unselected = not (test_targets target target_list) in
        if unselected && not (target_list = Var ("")) && verbose then
          begin Printf.printf "Info: settings '%s' unloaded for target '%s'\n" name target; flush stdout end;
        unselected && not (target_list = Var (""))
    | "module" ->
        let unselected = List.for_all (fun m -> m.file <> name) modules in
        if unselected && verbose then
          begin Printf.printf "Info: module '%s' unloaded for target '%s'\n" name target; flush stdout end
        else begin
          if verbose then
            (* display possible unloading of settings when the module itself is loaded *)
            List.iter (fun n ->
              let tag = Xml.tag n in
              let target_list = targets_of_field n "" in
              let valid = test_targets target target_list in
              if tag = "settings" && not (ExtXml.attrib_or_default n "target" "" = "") && not valid then
                begin Printf.printf "Info: settings of module '%s' unloaded for target '%s'\n" name target; flush stdout end;
            ) (Xml.children xml)
        end;
        unselected
    | _ -> false
  with _ -> false
