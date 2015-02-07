(*
 * A Label with a contrasting outline
 *
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
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

type icon = {
  lines : float array list;
  ellipse : float array list;
  width: int
}

val icon_fixedwing_template    : icon
val icon_flyingwing_template   : icon
val icon_rotorcraft_template   : icon
val icon_quadrotor_template    : icon
val icon_hexarotor_template    : icon
val icon_octorotor_template    : icon
val icon_quadrotor_x_template  : icon
val icon_hexarotor_x_template  : icon
val icon_octorotor_x_template  : icon
val icon_home_template         : icon

class widget :
  ?color : string ->
  ?icon_template : icon ->
  GnoCanvas.group ->
object
  method set_color : string -> unit
  method set_bg_color : string -> unit
end

