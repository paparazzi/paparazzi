(*
 * Geographic display
 *
 * Copyright (C) 2004-2008 ENAC, Pascal Brisset, Antoine Drouin
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

type projection = Mercator | UTM | LambertIIe
class type geographic = object method pos : Latlong.geographic end

class widget :
  ?height:int ->
  ?srtm:bool ->
  ?width:int ->
  ?projection:projection ->
  ?georef:Latlong.geographic ->
  unit ->
  object
    method add_info_georef : string -> < pos : Latlong.geographic > -> unit
    method edit_georef_name : string -> string -> unit
    method delete_georef : string -> unit
    method clear_georefs : unit -> unit
    method altitude : Latlong.geographic -> int
    method any_event : GdkEvent.any -> bool
    method arc :
      ?nb_points:int ->
      ?width:int ->
      ?fill_color:string ->
      float * float -> float -> float -> float -> GnoCanvas.line
    method background : GnoCanvas.group
    method background_event : GnoCanvas.item_event -> bool
    method maps : GnoCanvas.group array
    method canvas : GnoCanvas.canvas
    method center : Latlong.geographic -> unit
    method circle :
      ?group:GnoCanvas.group ->
      ?width:int ->
      ?fill_color:string ->
      ?opacity:int ->
      ?color:string -> Latlong.geographic -> Latlong.fmeter -> GnoCanvas.ellipse
    method convert_positions_to_points : Latlong.geographic array -> float array
    method connect_view : (unit -> unit) -> unit
    method current_zoom : float
    method display_alt : Latlong.geographic -> unit
    method display_geo : Latlong.geographic -> unit
    method display_group : string -> unit
    method display_pixbuf :
      ?opacity:int ->
      ?level:int ->
      (int * int) * Latlong.geographic ->
      (int * int) * Latlong.geographic -> GdkPixbuf.pixbuf -> GnoCanvas.pixbuf
    method display_xy : string -> unit
    method factory : GMenu.menu_shell GMenu.factory
    method file_menu : GMenu.menu
    method fit_to_window : unit -> unit
    method fix_bg_coords : Latlong.fmeter * Latlong.fmeter -> Latlong.fmeter * Latlong.fmeter
    method frame : GPack.box
    method georef : Latlong.geographic option
    method georefs : (string * < pos : Latlong.geographic >) list
    method get_center : unit -> Latlong.geographic
    method goto : unit -> unit
    method info : GPack.box
    method key_press : GdkEvent.Key.t -> bool
    method menubar : GMenu.menu_shell
    method mouse_motion : GdkEvent.Motion.t -> bool
    method move_item :
      ?z:float ->
      GnomeCanvas.re_p GnoCanvas.item -> Latlong.geographic -> unit
    method moveto : Latlong.geographic -> unit
    method of_world : Latlong.fmeter * Latlong.fmeter -> Latlong.geographic
    method pack_labels : unit
    method projection : string
    method photoprojection :
      ?group:GnoCanvas.group ->
      ?width:int ->
      ?fill_color:string ->
      ?color:string ->
      ?number:string -> Latlong.geographic -> Latlong.fmeter -> GnoCanvas.text
    method polygon :
      ?group:GnoCanvas.group ->
      ?width:int ->
      ?fill_color:string ->
      ?opacity:int ->
      ?color:string -> Latlong.geographic array -> GnoCanvas.polygon
    method pt2D_of : Latlong.geographic -> Geometry_2d.pt_2D
    method region : ((float * float) * (Latlong.fmeter * Latlong.fmeter)) option
    method register_to_fit : geographic -> unit
    method root : GnoCanvas.group
    method segment :
      ?group:GnoCanvas.group ->
      ?width:int ->
      ?fill_color:string -> Latlong.geographic -> Latlong.geographic -> GnoCanvas.line
    method set_georef : Latlong.geographic -> unit
    method set_utc_time : int -> int -> int -> unit
    method set_wind_sock : float -> string -> unit
    method still : GnoCanvas.group
    method switch_background : bool -> unit
    method switch_utc_time : bool -> unit
    method switch_utm_grid : bool -> unit
    method text :
      ?group:GnoCanvas.group ->
      ?fill_color:string ->
      ?x_offset:float ->
      ?y_offset:float -> Latlong.geographic -> string -> GnoCanvas.text
    method toolbar : GPack.box
    method top_still : float
    method utc_time : GnoCanvas.text
    method wind_sock : Wind_sock.item
    method window_to_world :
      winx:float -> winy:float -> Latlong.fmeter * Latlong.fmeter
    method world_of : Latlong.geographic -> Latlong.fmeter * Latlong.fmeter
    method zoom : float -> unit
    method zoom_adj : GData.adjustment
    method zoom_down : unit -> unit
    method zoom_in_place : float -> unit
    method zoom_in_center : float -> unit
    method zoom_up : unit -> unit
  end
