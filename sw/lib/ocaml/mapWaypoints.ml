(*
 * $Id$
 *
 * Waypoints objects
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

open Printf

let s = 5.
let losange = [|s;0.; 0.;s; -.s;0.; 0.;-.s|]

class group = fun ?(color="red") ?(editable=true) (geomap:MapCanvas.widget) ->
  let g = GnoCanvas.group geomap#canvas#root in
 object
    method group=g
    method geomap=geomap
    method color=color
   method editable=editable
end

class waypoint = fun (group:group) (name :string) ?(alt=0.) en ->
  let geomap=group#geomap
  and color = group#color
  and editable = group#editable in
  let xw, yw = geomap#world_of_en en in
  object (self)
    val mutable x0 = 0.
    val mutable y0 = 0.
    val item = 
      GnoCanvas.polygon group#group ~points:losange
	~props:[`FILL_COLOR color; `OUTLINE_COLOR "midnightblue" ; `WIDTH_UNITS 1.; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")]
	
    val label = GnoCanvas.text group#group ~props:[`TEXT name; `X s; `Y 0.; `ANCHOR `SW; `FILL_COLOR "green"]
    val mutable name = name
    val mutable alt = alt
    initializer self#move xw yw
    method name = name
    method set_name n =
      if n <> name then
	name <- n
    method alt = alt
    method label = label
    method xy = let a = item#i2w_affine in (a.(4), a.(5)) (*** item#i2w 0. 0. causes Seg Fault !***)
    method move dx dy = item#move dx dy; label#move dx dy
    method edit =
      let dialog = GWindow.window ~border_width:10 ~title:"Waypoint Edit" () in
      let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in
      let en = self#en in
      let ename  = GEdit.entry ~text:name ~packing:dvbx#add () in
      let ex  = GEdit.entry ~text:(string_of_float en.MapCanvas.east) ~packing:dvbx#add () in
      let ey  = GEdit.entry ~text:(string_of_float en.MapCanvas.north) ~packing:dvbx#add () in
      let ea  = GEdit.entry ~text:(string_of_float alt) ~packing:dvbx#add () in
      let cancel = GButton.button ~label:"Cancel" ~packing: dvbx#add () in 
      let ok = GButton.button ~label:"OK" ~packing: dvbx#add () in
      ignore(cancel#connect#clicked ~callback:dialog#destroy);
      ignore(ok#connect#clicked ~callback:
	       begin fun _ ->
		 self#set_name ename#text;
		 alt <- float_of_string ea#text;
		 label#set [`TEXT name];
		 self#set {MapCanvas.east = float_of_string ex#text;
			   north = float_of_string ey#text};
		 dialog#destroy ()
	       end);
      dialog#show ()



    method event (ev : GnoCanvas.item_event) =
      begin
	match ev with
	| `BUTTON_PRESS ev ->
	    begin
	      match GdkEvent.Button.button ev with
	      |	1 -> self#edit
	      |	3 -> self#delete
	      | 2 ->
		  let x = GdkEvent.Button.x ev
		  and y = GdkEvent.Button.y ev in
		  x0 <- x; y0 <- y;
		  let curs = Gdk.Cursor.create `FLEUR in
		  item#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs 
		    (GdkEvent.Button.time ev)
	      | x ->   printf "%d\n" x; flush stdout;
	    end
	| `MOTION_NOTIFY ev ->
	    let state = GdkEvent.Motion.state ev in
	    if Gdk.Convert.test_modifier `BUTTON2 state then begin
	      let x = GdkEvent.Motion.x ev
	      and y = GdkEvent.Motion.y ev in
	      let dx = geomap#current_zoom *. (x-. x0) 
	      and dy = geomap#current_zoom *. (y -. y0) in
	      self#move dx dy ;
	      x0 <- x; y0 <- y
	    end
	| `BUTTON_RELEASE ev ->
	    if GdkEvent.Button.button ev = 2 then
	      item#ungrab (GdkEvent.Button.time ev)
	| _ -> ()
      end;
      true
    initializer ignore(if editable then ignore (item#connect#event self#event))
    method item = item
    method en =
      let (dx, dy) = self#xy in
      geomap#en_of_world dx dy
    method set en = 
      let (xw, yw) = geomap#world_of_en en
      and (xw0, yw0) = self#xy in
      self#move (xw-.xw0) (yw-.yw0)      
    method delete =
      item#destroy ();
      label#destroy ()
    method zoom (z:float) =
      let a = item#i2w_affine in
      a.(0) <- 1./.z; a.(3) <- 1./.z; 
      item#affine_absolute a;
      label#affine_absolute a
    initializer self#zoom geomap#zoom_adj#value
    initializer ignore(geomap#zoom_adj#connect#value_changed (fun () -> self#zoom geomap#zoom_adj#value))
  end

let gensym = let n = ref 0 in fun prefix -> incr n; prefix ^ string_of_int !n

let waypoint = fun group ?(name = gensym "wp") ?alt en ->
  new waypoint group name ?alt en

