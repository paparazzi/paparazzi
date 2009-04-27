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

class pixmap_in_drawin_area = fun ~width ~height ~packing () -> 
  let da = GMisc.drawing_area ~width ~height ~show:true ~packing () in
  object
    val mutable pixmap = None

    method drawing_area = da

    method redraw = fun () ->
      match pixmap with
	  None -> ()
	| Some pm ->
	    (new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 pm#pixmap
	      
    method get_pixmap = fun () ->
      let {Gtk.width=width; height=height} = da#misc#allocation in
      let create = fun () -> GDraw.pixmap ~width ~height ~window:da () in
      let pm =
	match pixmap with
	  None -> create ()
	| Some pm ->
	    if pm#size = (width, height)
	    then pm
	    else begin 
	      Gdk.Pixmap.destroy pm#pixmap;
	      create ()
	    end in
      pixmap <- Some pm;
      pm
  end
