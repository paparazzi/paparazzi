(*
 * XML preprocessing tools
 *
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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
val nl : unit -> unit
val define : string -> string -> unit
val define_string : string -> string -> unit
val define_out : out_channel -> string -> string -> unit
val define_string_out : out_channel -> string -> string -> unit
val xml_error : string -> 'a
val sprint_float_array : string list -> string
val begin_out : out_channel -> string -> string -> unit
val start_and_begin_out : out_channel -> string -> string -> Xml.xml
val start_and_begin : string -> string -> Xml.xml
val begin_c_out : out_channel -> string -> string -> unit
val start_and_begin_c : string -> string -> Xml.xml
val finish : string -> unit
val finish_out : out_channel -> string -> unit
val warning : string -> unit
