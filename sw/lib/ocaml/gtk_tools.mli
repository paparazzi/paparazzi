(*
 * $Id$
 *
 * Lablgtk2 utils
 *  
 * Copyright (C) 2009 ENAC, Pascal Brisset
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
(** GTK utilities
 *)

(** Allocate a drawing area and filling pixmap on request *)
class pixmap_in_drawin_area : 
    width:int ->
    height:int ->
    packing:(GObj.widget -> unit) ->
    unit ->
      object
	method drawing_area : GMisc.drawing_area

	method get_pixmap : unit -> GDraw.pixmap
	(** Lazyly allocate a pixmap filling the drawing area *)

	method redraw : unit -> unit
        (** Redraw the pixmap *)
      end

