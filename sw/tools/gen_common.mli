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

val modules_dir : string

(** remove all duplicated elements of a list *)
val singletonize : 'a list -> 'a list

(** [get_modules_of_airframe xml]
 * Returns a list of modules ("load" node) from airframe file *)
val get_modules_of_airframe : Xml.xml -> Xml.xml list

(** [get_full_module_conf module] Parse module configuration file
 * Returns module file name and a pair (xml, xml list): parsed file, children *)
val get_full_module_conf : Xml.xml -> (string * (Xml.xml * Xml.xml list))

(** [get_module_conf module] Parse module configuration file
 * Returns parsed xml file *)
val get_module_conf : Xml.xml -> Xml.xml

(** [get_targets_of_module xml] Returns the list of targets of a module *)
val get_targets_of_module : Xml.xml -> string list

(** [unload_unused_modules modules ?print_error]
 * Returns a list of [modules] where unused modules are removed
 * If [print_error] is true, a warning is printed *)
val unload_unused_modules : Xml.xml list -> bool -> Xml.xml list

(** [get_modules_name xml]
 * Returns a list of loaded modules' name *)
val get_modules_name : Xml.xml -> string list

(** [targets_of_field]
 * Returns the targets of a makefile node in modules
 * Default "ap|sim" *)
val targets_of_field : Xml.xml -> string list

(** [get_modules_dir xml]
 * Returns the list of modules directories *)
val get_modules_dir : (Xml.xml * 'a) list -> string list

