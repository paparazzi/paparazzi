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
  | B921600
  | B1500000
  | B3000000

val speed_of_baudrate : string -> speed

val opendev : string -> speed -> bool -> Unix.file_descr
val close : Unix.file_descr -> unit
val set_dtr : Unix.file_descr -> bool -> unit
val set_speed : Unix.file_descr -> speed -> unit

val input :
  ?read:(Unix.file_descr -> bytes -> int -> int -> int) ->
  (bytes -> int) -> (Unix.file_descr -> unit) closure
(** Buffered input. [input ?read f] Returns a closure which must be called when
characters are available on the stream. These characters are stored in a
a buffer. [f] is then called on the buffer. [f] must return the number
of consumed characters. Default [read] is [Unix.read] *)

