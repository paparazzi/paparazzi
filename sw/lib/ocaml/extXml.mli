(*
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
(** [child xml ?p i] If [i] is an integer, returns the [i]'th (first is 0) child of [xml].
Else returns the child of [xml] with tag [i] (the first one satisfying [p]
if specified). Else raises [Not_found]. *)

val get : Xml.xml -> string -> Xml.xml
(** [get xml path] Returns the son of [xml] specified by [path] (where
separator is [.]). May raise [Not_found]. *)

val get_attrib : Xml.xml -> string -> string -> string
(** [get_attrib xml path attrib_name] *)

val attrib : Xml.xml -> string -> string
val int_attrib : Xml.xml -> string -> int
val float_attrib : Xml.xml -> string -> float
(** [get xml attribute_name] May raise [Error] *)

val tag_is : Xml.xml -> string -> bool
(** [tag_is xml s] Case safe test *)

val attrib_or_default : Xml.xml -> string -> string -> string
(** [get xml attribute_name default_value] *)

val to_string_fmt : ?tab_attribs:bool -> Xml.xml -> string
(** [to_string_fmt ?tab_attribs xml] Returns a formatted string where tag
    and attribute names are lowercase. [tab_attribs] requires attributes to
    be tabulated one by line *)

val subst_attrib : string -> string -> Xml.xml -> Xml.xml
(** [subst_attrib attrib_name new_value xml] *)

val subst_child :
    ?select:(Xml.xml -> bool) -> string -> Xml.xml -> Xml.xml -> Xml.xml
(** [subst_child ?select child_tag new_child xml] May raise Not_found *)

val subst_or_add_child : string -> Xml.xml -> Xml.xml -> Xml.xml
(** [subst_or_add_child child_tag new_child xml] *)

val remove_child :
    ?select:(Xml.xml -> bool) -> string -> Xml.xml -> Xml.xml
(** [delete_child ?select child_tag xml] Returns [xml] if not found *)

val parse_file : ?noprovedtd:bool -> string -> Xml.xml
(** Identical to Xml.parse_file with Failure exceptions. [nodtdprove] default is false. *)

val digest : Xml.xml -> Digest.t
(** Returns the MD5 digest of an XML tree (c.f. Digest module of std library) *)
val display_entities : string -> string

(** Conf à gconf handling *)
module Gconf : sig
  val get_value : Xml.xml -> string -> string
  val entry : string -> string -> string -> Xml.xml
  val add_entry : Xml.xml -> string -> string -> string -> Xml.xml
end
