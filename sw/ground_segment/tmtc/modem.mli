(*
 * Ground harware modem handling
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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


module Protocol :
  sig
    include Serial.PROTOCOL
    val stx : char
    val etx : int
    val payload_length : string -> int -> int
  end

val parse_payload : Serial.payload -> string option
(* Returns None for modem specific messages (while updating status) *)

type status = {
    mutable valim : float;
    mutable cd : int;
    mutable error : int;
    mutable debug : int;
    mutable nb_byte : int;
    mutable nb_msg : int;
    mutable nb_err : int;
    mutable detected : int
  }

val status : status


