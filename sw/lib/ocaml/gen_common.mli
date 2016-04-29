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

(* simple boolean expressions *)
type bool_expr =
  | Var of string
  | Not of bool_expr
  | And of bool_expr * bool_expr
  | Or of bool_expr * bool_expr

(* Module configuration:
  * Xml node
  * file (with path)
  * file name only
  * optional vpath
  * parameters
  * extrat targets
  *)
type module_conf = { name : string; xml : Xml.xml; file : string; filename : string; vpath : string option; param : Xml.xml list; targets : bool_expr; }

(* Modules directory *)
val modules_dir : string

(** remove all duplicated elements of a list *)
val singletonize : ?compare: ('a -> 'a -> int) -> 'a list -> 'a list

(** [targets_of_field] Xml node, default
 * Returns the targets expression of a makefile node in modules
 * Default "ap|sim" *)
val targets_of_field : Xml.xml -> string -> bool_expr

exception Subsystem of string
val module_name : Xml.xml -> string
val get_module : Xml.xml -> bool_expr -> module_conf

(** [get_modules_of_airframe xml]
 * Returns a list of pair (modules ("load" node), targets) from airframe file *)
val get_modules_of_airframe : ?target: string -> Xml.xml -> module_conf list

(** [get_modules_of_flight_plan xml]
 * Returns a list of module configuration from flight plan file *)
val get_modules_of_flight_plan : Xml.xml -> module_conf list

(** [get_modules_of_config ?target flight_plan airframe]
 * Returns a list of pair (modules ("load" node), targets) from airframe file and flight plan.
 * The modules are singletonized and options are merged *)
val get_modules_of_config : ?target:string -> ?verbose:bool -> Xml.xml -> Xml.xml -> module_conf list

(** [test_targets target targets]
 * Test if [target] is allowed [targets]
 * Return true if target is allowed, false if target is not in list or rejected (prefixed by !) *)
val test_targets : string -> bool_expr -> bool

(** [get_targets_of_module xml] Returns the boolean expression of targets of a module *)
val get_targets_of_module : Xml.xml -> bool_expr

(** [get_modules_name xml]
 * Returns a list of loaded modules' name *)
val get_modules_name : Xml.xml -> string list

(** [get_modules_dir xml]
 * Returns the list of modules directories *)
val get_modules_dir : module_conf list -> string list

(** [get_autopilot_of_airframe xml]
 * Returns (autopilot file, main freq) from airframe xml file
 * Raise Not_found if no autopilot
 * Fail if more than one *)
val get_autopilot_of_airframe : Xml.xml -> (string * string option)

(** [is_element_unselected target modules file]
 * Returns True if [target] is supported in the element [file] and, if it is
 * a module, that it is loaded,
 * [file] being the file name of an Xml file (module or setting) *)
val is_element_unselected : ?verbose:bool -> string -> module_conf list -> string -> bool

