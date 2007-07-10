(*
 * $Id$
 *
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

let (//) = Filename.concat

let paparazzi_src =
  try
    Sys.getenv "PAPARAZZI_SRC"
  with
    _ -> "/usr/share/paparazzi"

let paparazzi_home =
  try
    Sys.getenv "PAPARAZZI_HOME"
  with
    _ -> Filename.concat (Sys.getenv "HOME") "paparazzi"


let flight_plans_path = paparazzi_home // "conf" // "flight_plans"

let flight_plan_dtd = flight_plans_path // "flight_plan.dtd"

let icon_file = paparazzi_home // "data/pictures/penguin_icon.png"
