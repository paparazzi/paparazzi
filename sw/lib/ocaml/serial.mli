(*
 * Serial Port handling
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

type 'a closure = Closure of 'a

type speed =
    B0
  | B50
  | B75
  | B110
  | B134
  | B150
  | B200
  | B300
  | B600
  | B1200
  | B1800
  | B2400
  | B4800
  | B9600
  | B19200
  | B38400
  | B57600
  | B115200
  | B230400

val speed_of_baudrate : string -> speed

val opendev : string -> speed -> bool -> Unix.file_descr
val close : Unix.file_descr -> unit
val set_dtr : Unix.file_descr -> bool -> unit
val set_speed : Unix.file_descr -> speed -> unit

val input :
  ?read:(Unix.file_descr -> string -> int -> int -> int) ->
  (string -> int) -> (Unix.file_descr -> unit) closure
(** Buffered input. [input ?read f] Returns a closure which must be called when
characters are available on the stream. These characters are stored in a
a buffer. [f] is then called on the buffer. [f] must return the number
of consumed characters. Default [read] is [Unix.read] *)

type payload

val string_of_payload : payload -> string
val payload_of_string : string -> payload

exception Not_enough
module type PROTOCOL =
  sig
    val index_start : string -> int
    (** Must return the index of the first char of the the first message.
       May raise Not_found or Not_enough *)

    val length : string -> int -> int
    (** [length buf start] Must return the length of the message starting at
       [start]. May raise Not_enough *)

    val checksum : string -> bool
    (** [checksum message] *)

    val payload : string -> payload
    val packet : payload -> string
  end

(** Builds a parser module from a [PROTOCOL] description *)
module Transport :
  functor (Protocol : PROTOCOL) ->
    sig
      val nb_err : int ref (* Errors on checksum *)
      val discarded_bytes : int ref
      val parse : (payload -> unit) -> string -> int
      (** [parse f buf] Scans [buf] according to [Protocol] and applies [f] on
       payload of every recognised message. Returns the number of consumed bytes. *)
    end
