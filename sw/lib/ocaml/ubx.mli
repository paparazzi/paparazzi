(*
 * $Id$
 *
 * UBX protocol handling
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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
    val index_start : string -> int
    val payload_length : string -> int -> int
    val length : string -> int -> int
    val payload : string -> int -> string
    val uint8_t : int -> int
    val ( += ) : int ref -> int -> unit
    val checksum : string -> int -> string -> bool
  end

val nav_posutm : int * Xml.xml
val nav_status : int * Xml.xml
val nav_velned : int * Xml.xml
val send : out_channel -> int * Xml.xml -> (string * int) list -> unit
