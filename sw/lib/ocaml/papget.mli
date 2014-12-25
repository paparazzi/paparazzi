(*
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

class type value =
  object
    method last_value : string
    method connect : (string -> unit) -> unit
    method config : unit -> Xml.xml list
    method type_ : string
  end

class message_field :
  ?sender:string ->
  ?class_name:string ->
  string ->
  string ->
    value

class expression :
    ?extra_functions:(string * (string list -> string)) list ->
    ?sender:string ->
    Expr_syntax.expression ->
      value

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
  value ->
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

class canvas_video_plugin_item :
  Xml.xml list ->
  Papget_renderer.t ->
  object
    inherit canvas_item_type
    method config : unit -> Xml.xml
(*

    method connect : unit -> unit
    method deleted : bool
    method edit : unit -> unit
    method event : GnoCanvas.item_event -> bool
    method renderer : Papget_renderer.t
    method update : string -> unit
    method xy : float * float
*)
  end

