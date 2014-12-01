(*
* Widgets of the aircraft notebook
*
* Copyright (C) 2004-2006 ENAC, Pascal Brisset, Antoine Drouin
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

class alert : GBin.frame ->
  object
    method add : string -> unit
  end

class infrared : GBin.frame ->
  object
    method set_contrast_status : string -> unit
    method set_contrast_value : int -> unit
    method set_gps_hybrid_factor : float -> unit
    method set_gps_hybrid_mode : string -> unit
  end

class gps : ?visible:(GBin.frame -> bool) -> GBin.frame ->
  object
    method svsinfo : int -> (int*int*int*int) array -> unit
    method connect_reset : (int -> unit) -> unit
  end

class misc :
  packing:(GObj.widget -> unit) ->
  GBin.frame ->
  object
    method set_value : string -> string -> unit
    method periodic_send : bool
  end

type rc_mode = string
type rc_setting_mode = string
class rc_settings :
  ?visible:(GObj.widget -> bool) ->
  Xml.xml list ->
  object
    method set : float -> float -> unit
    method set_rc_mode : rc_mode -> unit
    method set_rc_setting_mode : rc_setting_mode -> unit
    method widget : GObj.widget
  end

type link_change = Linkup | Nochange | Linkdown
class link : ?visible:(GBin.frame -> bool) -> GBin.frame ->
  object
    method link_exists : string -> bool
    method add_link : string -> unit
    method update_link : string -> float -> float -> float -> int -> int -> link_change
    method links_ratio : unit -> (int * int)
    method multiple_links : unit -> bool
  end

