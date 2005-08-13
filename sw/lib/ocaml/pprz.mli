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
(** Message specification *)

val size_of_field : field -> int
val default_format : string -> string
val string_of_value : value -> string
type type_descr = {
    format : string ;
    glib_type : string;
    size : int;
    value : value
  }
val types : (string * type_descr) list
type values  = (string * value) list

val string_assoc : string -> values -> string
(** May raise Not_found *)

val float_assoc : string -> values -> float
(** May raise Not_found or Invalid_argument *)

exception Unknown_msg_name of string

module type CLASS = sig val name : string end
module Protocol : functor (Class : CLASS) -> sig
  include Serial.PROTOCOL
  val message_of_id : message_id -> message
  val message_of_name : string ->  message_id * message
  val values_of_payload : string -> message_id * values
  (** [values_of_bin payload] Parses a raw payload, returns the
   message id and the list of (field_name, value) *)
  val values_of_bin : string -> message_id * values
  (** [values_of_bin raw_message] Same than previous but [raw_message]
  includes header and checksum. *)
  val payload_of_values : message_id -> values -> string
  (** [payload_of_values m vs] Returns a payload *)


  val values_of_string : string -> message_id * values
  (** May raise [(Unknown_msg_name msg_name)] *)

  val string_of_message : message -> values -> string
  (** [string_of_message msg values] *)

  val message_send : string -> string -> values -> unit
  (** [message_send sender msg_name values] *)

  val message_bind : string -> (string -> values -> unit) -> Ivy.binding
  (** [message_bind msg_name callback] *)

  val message_answerer : string -> string -> (string -> values -> values) -> Ivy.binding
  (** [message_answerer sender msg_name callback] *)

  val message_req : string -> string -> values -> (string -> values -> unit) -> unit
  (** [message_answerer sender msg_name values receiver] Sends a request on the Ivy bus for the specified message. On reception, [receiver] will be applied on [sender_name] and expected values. *)
end
