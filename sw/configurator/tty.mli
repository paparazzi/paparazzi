(*
 *  $Id$
 *
 * Serial device handling
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

val connect : string -> unit
val deconnect : string -> unit
(** Opens and closes the given device *)

val add_ttyinput : string -> (string -> unit) -> unit
(** [add_ttyinput device cb] Attaches the callback [cb] to input events *)

val add_formatted_input : string -> string -> int -> (string -> unit) -> unit
(** [add_formatted_input device prefix size cb] Same as [add_ttyinput]
but [cb] is called only with an input of length [size] starting with
[prefix]. Characters are discarded until [prefix] is found. *)

val write : string -> string -> unit
val write_byte : string -> int -> unit
val flush : string -> unit
(** Output on the given device *)
