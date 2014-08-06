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

val flight_plans_path : string
val flight_plan_dtd : string

val icon_file : string
(** PNG paparazzi logo icon (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_mes_file : string
(** PNG paparazzi logo icon (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_gcs_file : string
(** PNG paparazzi logo icon (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val icon_rep_file : string
(** PNG paparazzi logo icon (48 x 48, 8-bit/color RGBA, non-interlaced) *)

val gconf_file : string
(** XML preferences file à la gconf *)

val gcs_icons_path : string

val expand_ac_xml : ?raise_exception:bool -> Xml.xml -> Xml.xml
(** Expands a conf.xml aircraft entry, adding the XML configuration files
(listed as attributes) as children. Returns an element containing the error
 message if raise_exception is false (default to true) *)
