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

type module_conf = { xml : Xml.xml; file : string; filename : string; vpath : string option; param : Xml.xml list; extra_targets : string list; }

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let modules_dir = paparazzi_conf // "modules"
let autopilot_dir = paparazzi_conf // "autopilot"

(** remove all duplicated elements of a list *)
let singletonize = fun l ->
  let rec loop = fun l ->
    match l with
        [] | [_] -> l
      | x::((x'::_) as xs) ->
        if x = x' then loop xs else x::loop xs in
  loop (List.sort compare l)

(** union of two lists *)
let union = fun l1 l2 ->
  let l = l1 @ l2 in
  let sl = List.sort compare l in
  singletonize sl

(** union of a list of list *)
let union_of_lists = fun l ->
  let sl = List.sort compare (List.flatten l) in
  singletonize sl

(** [targets_of_field]
    * Returns the targets of a makefile node in modules
    * Default "ap|sim" *)
let pipe_regexp = Str.regexp "|"
let targets_of_field = fun field default ->
  try
    Str.split pipe_regexp (ExtXml.attrib_or_default field "target" default)
  with
      _ -> []

(** [get_autopilot_of_airframe xml]
    * Returns (autopilot xml, main freq) from airframe xml file *)
let get_autopilot_of_airframe = fun xml ->
  (* extract all "autopilot" sections *)
  let section = List.filter (fun s -> compare (Xml.tag s) "autopilot" = 0) (Xml.children xml) in
  (* Raise error if more than one modules section *)
  match section with
      [autopilot] ->
        let freq = try Some (Xml.attrib autopilot "freq") with _ -> None in
        let ap = try Xml.attrib autopilot "name" with _ -> raise Not_found in
        (autopilot_dir // ap, freq)
    | [] -> raise Not_found
    | _ -> failwith "Error: you have more than one 'autopilot' section in your airframe file"

(** [get_modules_of_airframe xml]
    * Returns a list of module configuration from airframe file *)
let rec get_modules_of_airframe = fun xml ->
  (* extract all "modules" sections *)
  let section = List.filter (fun s -> compare (Xml.tag s) "modules" = 0) (Xml.children xml) in
  (* get autopilot file if any *)
  let ap_file = try
                  let (ap, _) = get_autopilot_of_airframe xml in
                  ap
    with _ -> "" in
  (* Raise error if more than one modules section *)
  match section with
      [modules] ->
      (* if only one section, returns a list of configuration *)
        let t_global = targets_of_field modules "" in
        let get_module = fun m t ->
          (* extract dir name if any and add paparazzi_home path if dir path is not global *)
          let (dir, vpath) = try
            let dir = ExtXml.attrib m "dir" in
            let dir = if Filename.is_relative dir then Env.paparazzi_home // dir else "" in
            (dir, Some dir)
          with _ -> (modules_dir, None) in
          let filename = ExtXml.attrib m "name" in
          let file = dir // filename in
          let targets = singletonize (t @ targets_of_field m "") in
          { xml = ExtXml.parse_file file; file = file; filename = filename; vpath = vpath; param = Xml.children m; extra_targets = targets }
        in
        let modules_list = List.map (fun m ->
          if compare (Xml.tag m) "load" <> 0 then Xml2h.xml_error "load";
          get_module m t_global
        ) (Xml.children modules) in
        let ap_modules = try
                           get_modules_of_airframe (ExtXml.parse_file ap_file)
          with _ -> [] in
        modules_list @ ap_modules
    | [] -> []
    | _ -> failwith "Error: you have more than one 'modules' section in your airframe file"

(** [get_targets_of_module xml]
    * Returns the list of targets of a module *)
let get_targets_of_module = fun conf ->
  let targets = List.map (fun x ->
    match String.lowercase (Xml.tag x) with
        "makefile" -> targets_of_field x Env.default_module_targets
      | _ -> []
  ) (Xml.children conf.xml) in
  let targets = (List.flatten targets) @ conf.extra_targets in
  (* return a singletonized list *)
  singletonize (List.sort compare targets)

(** [unload_unused_modules modules ?print_error]
    * Returns a list of [modules] where unused modules are removed
    * If [print_error] is true, a warning is printed *)
let unload_unused_modules = fun modules print_error ->
  let target = try Sys.getenv "TARGET" with _ -> "" in
  let is_target_in_module = fun m ->
    let target_is_in_module = List.exists (fun x -> String.compare target x = 0) (get_targets_of_module m) in
    if print_error && not target_is_in_module then
      Printf.fprintf stderr "Module %s unloaded, target %s not supported\n" (Xml.attrib m.xml "name") target;
    target_is_in_module
  in
  if String.length target = 0 then
    modules
  else
    List.find_all is_target_in_module modules

(** [get_modules_name xml]
    * Returns a list of loaded modules' name *)
let get_modules_name = fun xml ->
  (* extract all "modules" sections *)
  let modules = get_modules_of_airframe xml in
  (* filter the list if target is not supported *)
  let modules = unload_unused_modules modules false in
  (* return a list of modules name *)
  List.map (fun m -> ExtXml.attrib m.xml "name") modules

(** [get_modules_dir xml]
    * Returns the list of modules directories *)
let get_modules_dir = fun modules ->
  let dir = List.map (fun m -> try Xml.attrib m.xml "dir" with _ -> ExtXml.attrib m.xml "name") modules in
  singletonize (List.sort compare dir)

