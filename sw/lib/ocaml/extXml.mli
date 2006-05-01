(*
 * $Id$
 *
 * Xml-Light extension
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset
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

exception Error of string
val child : Xml.xml -> ?select:(Xml.xml -> bool) -> string -> Xml.xml
(** [child xml ?p i] If [i] is an integer, returns the [i]'th child of [xml].
Else returns the child of [xml] with tag [i] (the first one satisfying [p]
if specified). Else raises [Not_found]. *)

val get : Xml.xml -> string -> Xml.xml
(** [get xml path] Returns the son of [xml] specified by [path] (where
separator is [.]). May raise [Not_found]. *)

val get_attrib : Xml.xml -> string -> string -> string
(** [get_attrib xml path attrib_name] *)

val attrib : Xml.xml -> string -> string
(** [get xml attribute_name] May raise [Error] *)

val tag_is : Xml.xml -> string -> bool
(** [tag_is xml s] Case safe test *)

val attrib_or_default : Xml.xml -> string -> string -> string
(** [get xml attribute_name default_value] *)

val to_string_fmt : Xml.xml -> string
(** [to_string_fmt xml] Returns a formatted string where tag and attribute
names are lowercase *)

val subst_attrib : string -> string -> Xml.xml -> Xml.xml
val subst_child : string -> Xml.xml -> Xml.xml -> Xml.xml
val float_attrib : Xml.xml -> string -> float

