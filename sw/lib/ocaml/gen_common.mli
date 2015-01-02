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

(* Module configuration:
  * Xml node
  * file (with path)
  * file name only
  * optional vpath
  * parameters
  * extrat targets
  *)
type module_conf = { xml : Xml.xml; file : string; filename : string; vpath : string option; param : Xml.xml list; extra_targets : string list; }

(* Modules directory *)
val modules_dir : string

(** remove all duplicated elements of a list *)
val singletonize : 'a list -> 'a list

(** [targets_of_field] Xml node, default
 * Returns the targets of a makefile node in modules
 * Default "ap|sim" *)
val targets_of_field : Xml.xml -> string -> string list

(** [get_modules_of_airframe xml]
 * Returns a list of pair (modules ("load" node), targets) from airframe file *)
val get_modules_of_airframe : Xml.xml -> module_conf list

(** [get_targets_of_module xml] Returns the list of targets of a module *)
val get_targets_of_module : module_conf -> string list

(** [unload_unused_modules modules ?print_error]
 * Returns a list of [modules] where unused modules are removed
 * If [print_error] is true, a warning is printed *)
val unload_unused_modules : module_conf list -> bool -> module_conf list

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

