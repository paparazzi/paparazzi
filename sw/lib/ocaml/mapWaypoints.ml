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

module LL = Latlong
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

class waypoint = fun (group:group) (name :string) ?(alt=0.) wgs84 ->
  let geomap=group#geomap
  and color = group#color
  and editable = group#editable in
  let xw, yw = geomap#world_of wgs84 in
  let callbacks = Hashtbl.create 5 in
  let updated () =
    Hashtbl.iter (fun cb _ -> cb ()) callbacks in

  object (self)
    val mutable x0 = 0.
    val mutable y0 = 0.
    val item = 
      GnoCanvas.polygon group#group ~points:losange
	~props:[`FILL_COLOR color; `OUTLINE_COLOR "midnightblue" ; `WIDTH_UNITS 1.; `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001")]
	
    val label = GnoCanvas.text group#group ~props:[`TEXT name; `X s; `Y 0.; `ANCHOR `SW; `FILL_COLOR "green"]
    val mutable name = name (* FIXME: already in label ! *)
    val mutable alt = alt
    val mutable moved = false
    val mutable deleted = false
    initializer self#move xw yw
    method connect = fun (cb:unit -> unit) ->
      Hashtbl.add callbacks cb ()
    method name = name
    method set_name n =
      if n <> name then begin
	name <- n;
	label#set [`TEXT name]
      end
    method alt = alt
    method label = label
    method xy = let a = item#i2w_affine in (a.(4), a.(5)) (*** item#i2w 0. 0. causes Seg Fault !***)
    method move dx dy = item#move dx dy; label#move dx dy
    method edit =
      let dialog = GWindow.window ~border_width:10 ~title:"Waypoint Edit" () in
      let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in

      let wgs84 = self#pos in
      let s = sprintf "WGS84 %s" (geomap#geo_string wgs84) in
      let ename  = GEdit.entry ~text:name ~editable:false ~packing:dvbx#add () in
      let e_pos  = GEdit.entry ~text:s ~packing:dvbx#add () in
      let ea  = GEdit.entry ~text:(string_of_float alt) ~packing:dvbx#add () in

      let callback = fun _ ->
	self#set_name ename#text;
	alt <- float_of_string ea#text;
	label#set [`TEXT name];
	self#set (LL.of_string e_pos#text);
	updated ();
	dialog#destroy () in

      let cancel = GButton.button ~stock:`CANCEL ~packing: dvbx#add () in 
      ignore(cancel#connect#clicked ~callback:dialog#destroy);

      let ok = GButton.button ~stock:`OK ~packing: dvbx#add () in
      List.iter (fun e -> ignore (e#connect#activate ~callback)) 
	[ename; e_pos; ea];
      ok#grab_default ();
      ignore(ok#connect#clicked ~callback:dialog#destroy);
      dialog#show ()


    method event (ev : GnoCanvas.item_event) =
      begin
	match ev with
	| `BUTTON_PRESS ev ->
	    begin
	      match GdkEvent.Button.button ev with
	      |	2 -> self#edit
	      |	3 -> 
		  if (GToolbox.question_box ~title:"Confirm delete" ~buttons:["Cancel";"Delete"] ~default:2 (sprintf "Delete '%s' ?" name)) = 2 then
		    self#delete
	      | 1 ->
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
	    if Gdk.Convert.test_modifier `BUTTON1 state then begin
	      let x = GdkEvent.Motion.x ev
	      and y = GdkEvent.Motion.y ev in
	      let dx = geomap#current_zoom *. (x-. x0) 
	      and dy = geomap#current_zoom *. (y -. y0) in
	      self#move dx dy ;
	      updated ();
	      moved <- true;
	      x0 <- x; y0 <- y
	    end
	| `BUTTON_RELEASE ev ->
	    if GdkEvent.Button.button ev = 1 then begin
	      item#ungrab (GdkEvent.Button.time ev);
	      moved <- true
	    end
	| _ -> ()
      end;
      true
    initializer ignore(if editable then ignore (item#connect#event self#event))
    method moved = moved
    method reset_moved () = moved <- false
    method deleted = deleted
    method item = item
    method pos = geomap#of_world self#xy
    method set wgs84 = 
      let (xw, yw) = geomap#world_of wgs84
      and (xw0, yw0) = self#xy
      and z = geomap#zoom_adj#value in
      self#move ((xw-.xw0)*.z) ((yw-.yw0)*.z)
    method delete =
      deleted <- true; (* BOF *)
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

