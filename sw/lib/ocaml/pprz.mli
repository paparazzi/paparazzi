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

type sender_id = int
type class_id = int
type class_name = string
type message_id = int
type message_name = string
type packet_seq = int
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
external int64_of_bytes : string -> int -> int64 = "c_int64_of_indexed_bytes"
(** [int32_of_bytes buffer offset] *)

val separator : string
(** Separator in array values *)

val is_array_type : string -> bool
val is_fixed_array_type : string -> bool

val size_of_field : field -> int
val string_of_value : value -> string
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

val get_downlink_messages_in_one_class : Xml.xml -> Xml.xml
(** Messages.xml version handler. For those functions that are using stored messages.xml files.
 *  If the message is of the old version it returns the same file,
 *  if it's of the new version (2.0) it returns a 'protocol' xml element with one class called 'telemetry' including all downlink and datalink messages *)

val get_uplink_messages_in_one_class : Xml.xml -> Xml.xml
(** Messages.xml version handler. For those functions that are using stored messages.xml files.
 *  If the message is of the old version it returns the same file,
 *  if it's of the new version (2.0) it returns a 'protocol' xml element with one class called 'datalink' including all uplink and datalink messages *)

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

module type CLASS_NAME = sig
  val class_name : string
end

module type CLASS_TYPE = sig
  val class_type : string
end

type messages_mode = Type | Name

module type CLASS_Xml = sig
  val xml : Xml.xml
  val selection : string
  val mode : messages_mode
  val sel_class_id : int option
end

type msg_and_class_id = {
  msg_id : int;
  cls_id : int;
}

val current_protocol_version : string
val current_message_version : string

module type MESSAGES = sig
  val protocol_version : string
  val message_version : string
  val formated_xml : Xml.xml

  val messages : (msg_and_class_id, message) Hashtbl.t
  val message_of_id : ?class_id:int -> message_id -> message
  val message_of_name : string ->  message_id * message

  val class_id_of_msg : message_name -> class_id
  (** [class_id_of_msg msg_name] returns the class id containing the given message *)

  val class_id_of_msg_args : string -> class_id
  (** [class_id_of_msg_args args.(0)] returns the class id containing the given message when args.(0) is the parameter *)

  val class_id_of_msg_args_unsorted : string -> class_id
  (** [class_id_of_msg_args_unsorted args.(0)] returns the class id containing the given message when string with semicolons is the parameter *)

  val values_of_payload : Serial.payload -> packet_seq * sender_id * class_id * message_id * values
  (** [values_of_bin payload] Parses a raw payload, returns the
   the A/C id, class id, message id and the list of (field_name, value) *)

  val payload_of_values : ?gen_packet_seq:int -> sender_id -> ?class_id:int -> message_id -> values -> Serial.payload
  (** [payload_of_values ?gen_packet_seq sender_id class_id id vs] Returns a payload *)

  val values_of_string : string -> message_id * values
  (** May raise [(Unknown_msg_name msg_name)] *)

  val values_of_string_unsorted : string -> message_id * values
  (** May raise [(Unknown_msg_name msg_name)] *)

  val string_of_message : ?sep:string -> message -> values -> string
  (** [string_of_message ?sep msg values] Default [sep] is space *)

  val sort_values : string -> values -> values
  (** Sort the string of values with the xml order *)

  val message_send : ?timestamp:float -> string -> string -> values -> unit
  (** [message_send sender msg_name values] *)

  val message_bind : ?sender:string ->string -> (string -> values -> unit) -> Ivy.binding
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

module Messages_of_type : functor (Class : CLASS_TYPE) -> MESSAGES
module Messages_of_name : functor (Class : CLASS_NAME) -> MESSAGES

module MessagesOfXml : functor (Class : CLASS_Xml) -> MESSAGES
