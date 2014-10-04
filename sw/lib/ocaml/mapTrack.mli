(*
 * Track objects
 *
 * Copyright (C) 2004-2010 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

class track :
  ?name:string ->
  ?icon:string ->
  ?size:int ->
  ?color:string ->
  string ->
  MapCanvas.widget ->
  object
    method add_point : Latlong.geographic -> float -> unit
    method aircraft : GnoCanvas.group
    method clear : unit -> unit
    method clear_map2D : unit
    method clear_one : int -> unit
    method color : string
    method delete_desired_track : unit -> unit
    method draw_circle : Latlong.geographic -> Latlong.fmeter -> unit
    method draw_segment : Latlong.geographic -> Latlong.geographic -> unit
    method draw_zone : Latlong.geographic -> Latlong.geographic -> unit
    method height : unit -> float
    method incr : (Latlong.geographic * GnoCanvas.line) array -> unit
    method last : Latlong.geographic option
    method last_altitude : float
    method last_climb : float
    method last_heading : float
    method last_speed : float
    method move_cam : Latlong.geographic -> Latlong.geographic -> unit
    method move_carrot : Latlong.geographic -> unit
    method move_icon :
      Latlong.geographic -> float -> float -> float -> float -> unit
    method pos : Latlong.geographic
    method resize : int -> unit
    method set_cam_state : bool -> unit
    method set_color : string -> unit
    method set_label : string -> unit
    method set_last : Latlong.geographic option -> unit
    method set_params_state : bool -> unit
    method set_v_params_state : bool -> unit
    method size : int
    method track : GnoCanvas.group
    method update_ap_status : float -> unit
    method v_incr : (Latlong.geographic * float) array -> unit
    method v_path : (Latlong.geographic * float) array
    method zoom : float -> unit
    method event : GnoCanvas.item_event -> bool
    method set_event_cb : (string -> unit) -> unit
  end
