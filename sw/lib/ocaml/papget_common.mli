(*
 * Commons for papgets
 *
 * Copyright (C) 2008 ENAC
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

val get_property : string -> Xml.xml -> string
(** [get_property name config] *)

val get_prop : string -> Xml.xml list -> string -> string
(** [get_prop name config_list default_value] *)

val property : string -> string -> Xml.xml
val xml : string -> string -> (string * string) list -> Xml.xml
val float_property : string -> float -> Xml.xml
val dnd_source : GObj.widget -> Xml.xml -> unit
