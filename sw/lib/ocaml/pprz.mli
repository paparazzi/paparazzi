(*
 * $Id$
 *
 * Downlink protocol (handling messages.xml)
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

type class_name = string
type message_id = int
type format = string
type _type = string
type value = Int of int | Float of float | String of string | Int32 of int32
type field = { _type : _type; fformat : format; }
type message = { name : string; fields : (string * field) list; }
val size_of_field : field -> int
val default_format : string -> string
val string_of_value : value -> string
type type_descr = {
    format : string ;
    glib_type : string;
    size : int;
    value : string
  }
val types : (string * type_descr) list

exception Unknown_msg_name of string

module type CLASS = sig val name : string end
module Protocol : functor (Class : CLASS) -> sig
  include Serial.PROTOCOL
  val message_of_id : message_id -> message
  val message_of_name : string ->  message_id * message
  val values_of_bin : string -> message_id * (string * value) list
(** [values raw_message] Parses a raw message, returns the
   message id and the liste of (field_name, value) *)

  val values_of_string : string -> message_id * (string * value) list
  (** May raise [(Unknown_msg_name msg_name)] *)

  val string_of_message : message -> (string * value) list -> string
  (** [string_of_message msg values] *)
end
    

