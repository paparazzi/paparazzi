(*
 * UBX protocol handling
 *
 * Copyright (C) 2004-2006 Pascal Brisset, Antoine Drouin
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

module UbxProtocol :  Protocol.PROTOCOL

type message_spec

type class_id = int
type msg_id = int

val message : string -> string -> class_id*msg_id*message_spec
val ubx_payload : message_spec -> (string * int) list -> string
val payload : string -> string -> (string * int) list -> Protocol.payload
