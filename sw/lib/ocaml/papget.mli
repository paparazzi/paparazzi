(*
 * $Id$
 *
 * Paparazzi widgets
 *  
 * Copyright (C) 2008 ENAC
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

class type item =
  object
    method config : unit -> Xml.xml
    method deleted : bool
  end

class message :
  ?sender:string ->
  ?class_name:string ->
  string ->
  object
    method connect : string -> (string -> unit) -> unit
    method msg_name : string
  end


class type canvas_item_type = 
  object
    method connect : unit -> unit
    method deleted : bool
    method edit : unit -> unit
    method event : GnoCanvas.item_event -> bool
    method renderer : Papget_renderer.t
    method update : string -> unit
    method xy : float * float
  end

class canvas_display_float_item :
  config:Xml.xml list ->
  message ->
  string ->
  Papget_renderer.t ->
  object
    inherit canvas_item_type

    method config : unit -> Xml.xml
    method connect : unit -> unit
    method update_field : string -> unit
  end

class canvas_goto_block_item :
  Xml.xml list ->
  (unit -> unit) ->
  Papget_renderer.t ->
  object
    method config : unit -> Xml.xml
    method connect : unit -> unit
    method deleted : bool
    method edit : unit -> unit
    method event : GnoCanvas.item_event -> bool
    method renderer : Papget_renderer.t
    method update : string -> unit
    method xy : float * float
  end

class canvas_variable_setting_item :
  Xml.xml list ->
  (unit -> unit) ->
  Papget_renderer.t ->
  object
    method config : unit -> Xml.xml
    method connect : unit -> unit
    method deleted : bool
    method edit : unit -> unit
    method event : GnoCanvas.item_event -> bool
    method renderer : Papget_renderer.t
    method update : string -> unit
    method xy : float * float
  end
