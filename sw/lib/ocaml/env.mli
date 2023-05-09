(*
 * Configuration handling
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

val paparazzi_home : string
(** User's files directory *)

val paparazzi_src : string
(** Installation's files directory *)

val paparazzi_conf : string

val flight_plans_path : string
val flight_plan_dtd : string

val modules_paths : string list
val modules_ext_paths : string list

val srtm_path : string
val srtm_pprzgcs_path : string

val icon_file : string
(** PNG paparazzi logo icon (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_mes_file : string
(** PNG paparazzi logo icon for messages (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_gcs_file : string
(** PNG paparazzi logo icon for GCS (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_rep_file : string
(** PNG paparazzi logo icon for replay (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_rtp_file : string
(** PNG paparazzi logo icon for RT plotter (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_log_file : string
(** PNG paparazzi logo icon for log plotter (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_sim_file : string
(** PNG paparazzi logo icon for simulator (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val gconf_file : string
(** XML preferences file à la gconf *)

val gcs_icons_path : string
val gcs_default_icons_theme : string
val get_gcs_icon_path : string -> string -> string
(** get_gcs_icon_path theme icon
 * Get gcs icon path from icon name and theme name
 * fallback to default theme or raise Not_found *)

(* Default targets for modules *)
val default_module_targets : string

val dump_fp : string
(** path to flight plan dump tool *)

val filter_absolute_path : string -> string
(** remove absolute path paparazzi_home/conf if it exists
 *  returns a relative path *)

val filter_settings : string -> string
(** filter settings (a string separted by white spaces)
 *  and keep the ones without brackets
 *  (return a string of filtered name separate by white spaces) *)

val get_paparazzi_version : unit -> string
(** read the current paparazzi_version *)

val key_modifiers_of_string : string -> string
(** Convert key modifiers from Qt style (without '<' or '>', separated with '+')
 *  to GTK style.
 *  Supported modifiers are Alt, Ctrl, Shift and Meta
 *)

