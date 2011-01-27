(*
 * $Id$
 *
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

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let modules_dir = paparazzi_conf // "modules"

let default_module_targets = "ap|sim"

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

(** [get_modules_of_airframe xml]
 * Returns a list of pair (modules ("load" node), targets) from airframe file *)
let get_modules_of_airframe = fun xml ->
  (* extract all "modules" sections *)
  let modules = List.map (fun x ->
    match String.lowercase (Xml.tag x) with
      "modules" ->
        let targets = targets_of_field x "" in
        List.map (fun m -> (m,targets)) (Xml.children x)
    | _ -> []
    ) (Xml.children xml) in
  (* flatten the list (result is a list of "load" xml nodes) *)
  List.flatten modules

(** [get_full_module_conf module] Parse module configuration file (with extra targets)
 * Returns module file name and a triple (xml, xml list, targets): parsed file, children, extra targets *)
let get_full_module_conf = fun (m, t) ->
  match Xml.tag m with
    "load" -> let file = modules_dir // ExtXml.attrib m "name" in
      let targets = targets_of_field m "" in
      (file, (ExtXml.parse_file file, Xml.children m, t @ targets))
  | _ -> Xml2h.xml_error "load"

(** [get_module_conf module] Parse module configuration file
 * Returns parsed xml file *)
let get_module_conf = fun m ->
  let (_ , (conf, _, _)) = get_full_module_conf (m, []) in
  conf

(** [get_targets_of_module xml] Returns the list of targets of a module *)
let get_targets_of_module = fun m ->
  let targets = List.map (fun x ->
    match String.lowercase (Xml.tag x) with
      "makefile" -> targets_of_field x default_module_targets
    | _ -> []
  ) (Xml.children m) in
  (* return a singletonized list *)
  singletonize (List.sort compare (List.flatten targets))

(** [unload_unused_modules modules ?print_error]
 * Returns a list of [modules] where unused modules are removed
 * If [print_error] is true, a warning is printed *)
let unload_unused_modules = fun modules print_error ->
  let target = try Sys.getenv "TARGET" with _ -> "" in
  let is_target_in_module = fun m ->
    let target_is_in_module = List.exists (fun x -> String.compare target x = 0) (get_targets_of_module m) in
    if print_error && not target_is_in_module then
      Printf.fprintf stderr "Module %s unloaded, target %s not supported\n" (Xml.attrib m "name") target;
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
  (* parse modules *)
  let modules = List.map (fun (m,_) -> ExtXml.parse_file (modules_dir // ExtXml.attrib m "name")) modules in
  (* filter the list if target is not supported *)
  let modules = unload_unused_modules modules false in
  (* return a list of modules name *)
  List.map (fun m -> ExtXml.attrib m "name") modules

(** [get_targets_of_module xml]
 * Returns the list of targets of a module *)
let get_targets_of_module = fun m ->
  let targets = List.map (fun x ->
    match String.lowercase (Xml.tag x) with
      "makefile" -> targets_of_field x default_module_targets
    | _ -> []
  ) (Xml.children m) in
  (* return a singletonized list *)
  singletonize (List.sort compare (List.flatten targets))

(** [get_modules_dir xml]
 * Returns the list of modules directories *)
let get_modules_dir = fun modules ->
  let dir = List.map (fun (m, _, _) -> try Xml.attrib m "dir" with _ -> ExtXml.attrib m "name") modules in
  singletonize (List.sort compare dir)

