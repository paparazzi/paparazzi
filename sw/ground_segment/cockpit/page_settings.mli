(*
* Widget to pack settings buttons
*
* Copyright (C) 2004-2009 ENAC, Pascal Brisset, Antoine Drouin
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

(** [new Page_settings.settings ?visible dl_settings callback short_button_receiver] *)
class settings : ?visible:(GObj.widget -> bool) -> Xml.xml list -> (int -> float -> unit) -> string -> string -> (string -> GObj.widget -> unit) ->
  object
    method length : int (** Total number of settings *)
    method set : int -> string option -> unit (** Set the current value *)
    method assoc : string -> int
    method widget : GObj.widget
    method save : string -> unit
	(** [save airframe_filename] *)
    method keys : (Gdk.keysym * (Gdk.Tags.modifier list * (unit -> unit))) list
	(** (key, (modifiers, action)) list *)
  end

