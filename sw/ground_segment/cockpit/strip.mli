(*
 * Strip handling
 *
 * Copyright (C) 2006-2009 ENAC, Pascal Brisset, Antoine Drouin
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

type t = <
    add_widget : ?group:string -> GObj.widget -> unit;
    (** Add a user widget in the low row of the strip *)

    connect_shift_alt : (float -> unit) -> unit;
    connect_shift_lateral : (float -> unit) -> unit;
    connect_launch : (float -> unit) -> unit;
    connect_kill : bool -> (float -> unit) -> unit;
    connect_mode : float -> (float -> unit) -> unit;
    connect_flight_time : (float -> unit) -> unit;

    connect_apt : (unit -> float) -> (float -> unit) -> unit;
    (** [connect_apt get_ac_unix_time send_value] *)

    connect_mark : (unit -> unit) -> unit;
    set_agl : float -> unit;
    set_bat : float -> unit;
    set_throttle : ?kill:bool -> float -> unit;
    set_speed : float -> unit;
    set_airspeed : float -> unit;
    set_climb : float -> unit;
    set_color : string -> string -> unit;
    set_label : string -> string -> unit;
    set_rc : int -> string -> unit;
    hide_buttons : unit -> unit;
    show_buttons : unit -> unit;
    connect : (unit -> unit) -> unit
>

type strip_param = {
  color : string;
  min_bat : float;
  max_bat : float;
  alt_shift_plus_plus : float;
  alt_shift_plus : float;
  alt_shift_minus : float;
  icons_theme : string; }


val add : PprzLink.values -> strip_param -> GPack.box -> t
(** [add config params] *)
