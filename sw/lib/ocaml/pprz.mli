(*
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

val messages_xml : unit -> Xml.xml

type class_name = string
type message_id = int
type ac_id = int
type format = string
type _type =
    Scalar of string
  | ArrayType of string
  | FixedArrayType of string * int
type value =
    Int of int | Float of float | String of string | Int32 of int32 | Char of char | Int64 of int64
  | Array of value array
type field = {
    _type : _type;
    fformat : format;
    alt_unit_coef : string; (* May be empty *)
    enum : string list (* 'values' attribute *)
  }
type link_mode = Forwarded | Broadcasted
type message = {
    name : string;
    fields : (string * field) list;
    link : link_mode option
  }
(** Message specification *)


external int32_of_bytes : string -> int -> int32 = "c_int32_of_indexed_bytes"
external uint32_of_bytes : string -> int -> int64 = "c_uint32_of_indexed_bytes"
external int64_of_bytes : string -> int -> int64 = "c_int64_of_indexed_bytes"
(** [int32_of_bytes buffer offset] *)

val separator : string
(** Separator in array values *)

val is_array_type : string -> bool
val is_fixed_array_type : string -> bool

val size_of_field : field -> int
val string_of_value : value -> string
val formatted_string_of_value : format -> value -> string
val int_of_value : value -> int (* May raise Invalid_argument *)
type type_descr = {
    format : string ;
    glib_type : string;
    inttype : string;
    size : int;
    value : value
  }
val types : (string * type_descr) list
type values  = (string * value) list

val assoc : string -> values -> value
(** Safe assoc taking into accound characters case. May raise Failure ... *)

val string_assoc : string -> values -> string
(** May raise Not_found *)

val float_assoc : string -> values -> float
val int_assoc : string -> values -> int
val int32_assoc : string -> values -> Int32.t
val uint32_assoc : string -> values -> Int64.t
val int64_assoc : string -> values -> Int64.t
(** May raise Not_found or Invalid_argument *)

val hex_of_int_array : value -> string
(** Returns the hexadecimal string of an array of integers *)

exception Unit_conversion_error of string
(** Unit_conversion_error raised when parsing error occurs *)
exception Unknown_conversion of string * string
(** Unknown_conversion raised when conversion fails *)
exception No_automatic_conversion of string * string
(** No_automatic_conversion raised when no conversion found
 *  and from_unit or to_unit are empty string
 *)

val scale_of_units : ?auto:string -> string -> string -> float
(** scale_of_units from to
 *  Returns conversion factor between two units
 *  The possible conversions are described in conf/units.xml
 *  May raise Invalid_argument if one of the unit is not valid
 *  or if units.xml is not valid
 *)

val alt_unit_coef_of_xml : ?auto:string -> Xml.xml -> string
(** Return coef for alternate unit
 *)

val key_modifiers_of_string : string -> string
(** Convert key modifiers from Qt style (without '<' or '>', separated with '+')
 *  to GTK style.
 *  Supported modifiers are Alt, Ctrl, Shift and Meta
 *)

exception Unknown_msg_name of string * string
(** [Unknown_msg_name (name, class_name)] Raised if message [name] is not
found in class [class_name]. *)

module Transport : Serial.PROTOCOL
(** Pprz frame (sw/airborne/pprz_transport.h):
    |STX|length|... payload=(length-4) bytes ...|Checksum A|Checksum B|
    Where checksum is computed over length and payload:
    ck_A = ck_B = 0;
    for all byte b in payload
      ck_A += b; ck_b += ck_A

    STX = 0x99
    [packet] raises Invalid_Argument if length >= 256
 *)

module TransportExtended : Serial.PROTOCOL
(** Pprz frame (sw/airborne/pprz_transport.h):
    |STX|length|timestamp|... payload=(length-4) bytes ...|Checksum A|Checksum B|
    Where checksum is computed over length and payload:
    ck_A = ck_B = 0;
    for all byte b in payload
      ck_A += b; ck_b += ck_A

    STX = 0x98
    [packet] raises Invalid_Argument if length >= 256
 *)

val offset_fields : int

module type CLASS = sig
  val name : string
end

module type CLASS_Xml = sig
  val xml : Xml.xml
  val name : string
end

module type MESSAGES = sig
  val messages : (message_id, message) Hashtbl.t
  val message_of_id : message_id -> message
  val message_of_name : string ->  message_id * message

  val values_of_payload : Serial.payload -> message_id * ac_id * values
  (** [values_of_bin payload] Parses a raw payload, returns the
   message id, the A/C id and the list of (field_name, value) *)

  val payload_of_values : message_id -> ac_id -> values -> Serial.payload
  (** [payload_of_values id ac_id vs] Returns a payload *)

  val values_of_string : string -> message_id * values
  (** May raise [(Unknown_msg_name msg_name)] *)

  val string_of_message : ?sep:string -> message -> values -> string
  (** [string_of_message ?sep msg values] Default [sep] is space *)

  val message_send : ?timestamp:float -> ?link_id:int -> string -> string -> values -> unit
  (** [message_send sender msg_name values] *)

  val message_bind : ?sender:string -> ?timestamp:bool -> string -> (string -> values -> unit) -> Ivy.binding
  (** [message_bind ?sender msg_name callback] *)

  val message_answerer : string -> string -> (string -> values -> values) -> Ivy.binding
  (** [message_answerer sender msg_name callback] Set a handler for a
      [message_req] (which will send a [msg_name]_REQ message).
      [callback asker args] must return the list of attributes of the answer. *)

  val message_req : string -> string -> values -> (string -> values -> unit) -> unit
  (** [message_req sender msg_name values receiver] Sends a request on the Ivy
      bus for the specified message. A [msg_name]_REQ message is send and a
      [msg_name] message is expected for the reply. On reception, [receiver]
      will be applied on [sender_name] and attribute values of the values. *)
end

module Messages : functor (Class : CLASS) -> MESSAGES
module MessagesOfXml : functor (Class : CLASS_Xml) -> MESSAGES
