(*
 * Debugging facilities
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

(** Module for outputting debug information *)

val log : out_channel ref
(** The log file. Default value is "debug_log". The debug output channel can
    be changed by [Debug.log := out_channel].
    The output channel gets back to its default value after each call to
    [Debug.call] *)

val call : char -> (out_channel -> unit) -> unit
(** [call debug_level output_function] outputs a message to the current
    channel [!Debug.log]
    [debug_level] is a flag to categorize the debug message.
    The environment variable PPRZ_DEBUG contains the active debug levels.
    [call debug_level output_function] outputs its message iff PPRZ_DEBUG
    contains the character [debug_level] or [*]
    [output_function]
    [Debug.call 'x' (fun c -> Printf.fprintf c "message")] outputs ["message"]
    iff the flag 'x' is active (ie PPRZ_DEBUG contains the character ['x']) *)

(** No debug information is output if the program was compiled with the
    -noassert flag *)

val trace : char -> string -> unit

val xprint : string -> string
(** Returns the hexadecimal representation of a string *)
