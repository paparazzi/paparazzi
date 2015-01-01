(*
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
open LL

module CL = ContrastLabel

(*
 * Waypoint label offsets
 *)
let s = 6. (* x offset *)

class group = fun ?(color="red") ?(editable=true) ?(show_moved=false) (geomap:MapCanvas.widget) ->
  let g = GnoCanvas.group geomap#canvas#root in
object
  method group=g
  method geomap=geomap
  method color=color
  method editable=editable
  method show_moved = show_moved
end

let rotation_45 =
  let s = sin (Latlong.pi/.4.) in
  [|s;s;-.s;s;0.;0.|]

class waypoint = fun ?(show = true) (wpts_group:group) (name :string) ?(alt=0.) wgs84 ->
  let geomap=wpts_group#geomap
  and color = wpts_group#color
  and editable = wpts_group#editable in
  let xw, yw = geomap#world_of wgs84 in
  let callbacks = Hashtbl.create 5 in
  let updated () =
    Hashtbl.iter (fun cb _ -> cb ()) callbacks in

  let wpt_group = GnoCanvas.group wpts_group#group in

  let item =
    GnoCanvas.rect wpt_group ~x1:(-.s) ~y1:(-.s) ~x2:s ~y2:s ~props:[`FILL_COLOR color; `OUTLINE_COLOR "black"] in

  let anim = function
  None ->
    Some (Glib.Timeout.add 500 (fun () -> Gdk.X.beep (); item#affine_relative rotation_45; true))
    | Some x -> Some x in


object (self)
  val mutable x0 = 0.
  val mutable y0 = 0.

  val label = new CL.widget ~name:name ~color:"white" s 0. wpt_group
  val mutable name = name (* FIXME: already in label ! *)
  val mutable alt = alt
  val mutable ground_alt = 0.
  val mutable moved = None
  val mutable deleted = false
  val mutable commit_cb = None
  initializer
    if not show then wpt_group#hide ()
  initializer
    item#affine_absolute rotation_45;
    self#move xw yw
  method connect = fun (cb:unit -> unit) ->
    Hashtbl.add callbacks cb ()
  method set_commit_callback = fun (cb:unit -> unit) -> commit_cb <- Some cb
  method name = name
  method set_name n =
    if n <> name then begin
      name <- n;
      label#set_name name
    end
  method geomap = geomap
  method alt = alt
  method label = label
  method xy = let a = wpt_group#i2w_affine in (a.(4), a.(5))
  method move dx dy =
    wpt_group#move dx dy;
    wpt_group#raise_to_top ()
  method edit =
    let dialog = GWindow.window ~type_hint:`DIALOG ~modal:true ~position:`MOUSE ~border_width:10 ~title:"Waypoint Edit" () in
    let dvbx = GPack.box `VERTICAL ~packing:dialog#add () in

    let ename  = GEdit.entry ~text:name ~editable ~packing:dvbx#add () in
    let hbox = GPack.hbox ~packing:dvbx#add () in

    let optmenu = GMenu.option_menu ~packing:hbox#add () in
    let e_pos  = GEdit.entry ~width_chars:25 ~packing:hbox#add () in

      (* We would like to share the menu of the map: it does not work ! *)
    let selected_georef = ref WGS84_dec in
    let display_coordinates = fun () ->
      e_pos#set_text (string_of_coordinates !selected_georef self#pos)
    and set_coordinates = fun () ->
      self#set (geographic_of_coordinates !selected_georef e_pos#text) in

    display_coordinates ();

    let initial_wgs84 = self#pos in

    let menu = GMenu.menu () in
    let set = fun kind () ->
      set_coordinates ();
      selected_georef := kind;
      display_coordinates () in
    let mi = GMenu.menu_item ~label:"WGS84" ~packing:menu#append () in
    ignore (mi#connect#activate ~callback:(set WGS84_dec));
    let mi = GMenu.menu_item ~label:"WGS84_dms" ~packing:menu#append () in
    ignore (mi#connect#activate ~callback:(set WGS84_dms));
    let mi = GMenu.menu_item ~label:"LambertIIe" ~packing:menu#append () in
    ignore (mi#connect#activate ~callback:(set LBT2e));
    List.iter (fun (label, geo) ->
      let mi = GMenu.menu_item ~label ~packing:menu#append () in
      ignore (mi#connect#activate ~callback:(set (Bearing geo))))
      geomap#georefs;
    optmenu#set_menu menu;

    let ha = GPack.hbox ~packing:dvbx#add () in
    let minus10= GButton.button ~label:"-10" ~packing:ha#add () in
      (*      let ea  = GEdit.entry ~text:(string_of_float alt) ~packing:ha#add () in *)
    let ea = GEdit.spin_button  ~rate:0. ~digits:2 ~width:50 ~packing:ha#add ()
    and adj = GData.adjustment
      ~value:alt ~lower:(-100.) ~upper:10000.
      ~step_incr:1. ~page_incr:10.0 ~page_size:0. () in
    ea#set_adjustment adj;
    ea#set_value alt; (* this should be done by set_adjustment but seems to fail on ubuntu 13.10 (at least) *)

    let agl = alt -. (try float (Srtm.of_wgs84 initial_wgs84) with _ -> ground_alt) in
    let agl_lab  = GMisc.label ~text:(sprintf " AGL: %4.0fm" agl) ~packing:ha#add () in
    let plus10= GButton.button ~label:"+10" ~packing:ha#add () in
    let change_alt = fun x ->
      ea#set_value (ea#value +. x) in
    ignore(minus10#connect#pressed (fun _ -> change_alt (-10.)));
    ignore(plus10#connect#pressed (fun _ -> change_alt (10.)));

    (* called when ok button is clicked in WP Edit dialog *)
    let callback = fun _ ->
      geomap#edit_georef_name name ename#text;
      self#set_name ename#text;
      alt <- ea#value;
      label#set_name name;
      set_coordinates ();
      updated ();
      if wpts_group#show_moved then
        moved <- anim moved;
      begin
        match commit_cb with
            Some cb -> cb ()
          | None -> ()
      end;
      dialog#destroy ()
    in
    let dhbx = GPack.box `HORIZONTAL ~packing: dvbx#add () in

    let cancel = GButton.button ~stock:`CANCEL ~packing: dhbx#add () in
    let destroy = fun () ->
      self#set initial_wgs84;
      self#reset_moved ();
      wpt_group#lower_to_bottom ();
      dialog#destroy () in
    ignore(cancel#connect#clicked ~callback:destroy);

      (** Delete button for editable waypoints *)
    if editable then begin
      let delete = GButton.button ~stock:`DELETE ~packing: dhbx#add () in
      let delete_callback = fun () ->
        dialog#destroy ();
        self#delete ();
        geomap#delete_georef name;
        updated ()
      in
      ignore(delete#connect#clicked ~callback:delete_callback)
    end;

    let ok = GButton.button ~stock:`OK ~packing: dhbx#add () in
    List.iter
      (fun e -> ignore (e#connect#activate ~callback))
      [ename; e_pos];
    ok#grab_default ();

    ignore(ok#connect#clicked ~callback:(fun _ -> callback (); dialog#destroy ()));

      (* Update AGL on pos or alt change *)
    let callback = fun _ ->
      try
        set_coordinates ();
        let wgs84 = self#pos in
        let agl  = ea#value -. (try float (Srtm.of_wgs84 wgs84) with _ -> ground_alt) in
        agl_lab#set_text (sprintf " AGL: %4.0fm" agl)
      with _ -> ()
    in
    ignore (ea#connect#changed ~callback);
    ignore (e_pos#connect#changed ~callback);
    dialog#show ()

  val mutable motion = false
  method event (ev : GnoCanvas.item_event) =
    begin
      match ev with
        | `BUTTON_PRESS ev ->
          begin
            match GdkEvent.Button.button ev with
              | 1 ->
                motion <- false;
                let x = GdkEvent.Button.x ev
                and y = GdkEvent.Button.y ev in
                x0 <- x; y0 <- y;
                let curs = Gdk.Cursor.create `FLEUR in
                item#grab [`POINTER_MOTION; `BUTTON_RELEASE] curs
                  (GdkEvent.Button.time ev)
              | _ -> ()
          end
        | `MOTION_NOTIFY ev ->
          let state = GdkEvent.Motion.state ev in
          if Gdk.Convert.test_modifier `BUTTON1 state then begin
            motion <- true;
            let x = GdkEvent.Motion.x ev
            and y = GdkEvent.Motion.y ev in
            let dx = geomap#current_zoom *. (x-. x0)
            and dy = geomap#current_zoom *. (y -. y0) in
            self#move dx dy ;
            updated ();
            if wpts_group#show_moved then
              moved <- anim moved;
            x0 <- x; y0 <- y
          end
        | `BUTTON_RELEASE ev ->
          if GdkEvent.Button.button ev = 1 then begin
            item#ungrab (GdkEvent.Button.time ev);
            self#edit
          end
        | _ -> ()
    end;
    true
  initializer ignore(item#connect#event self#event)
  method moved = moved <> None
  method reset_moved () =
    match moved with
      | None -> ()
      | Some x ->
        Glib.Timeout.remove x;
        item#affine_absolute rotation_45;
        moved <- None

  method deleted = deleted
  method item = item
  method pos = geomap#of_world self#xy
  method set ?altitude ?(update=false) wgs84 =
    let (xw, yw) = geomap#world_of wgs84
    and (xw0, yw0) = self#xy
    and z = geomap#zoom_adj#value in

    let dx = (xw-.xw0)*.z
    and dy = (yw-.yw0)*.z
    and dz = match altitude with Some a -> a -. alt | _ -> 0. in

    let current_ecef = ecef_of_geo WGS84 self#pos self#alt
    and new_ecef = ecef_of_geo WGS84 wgs84 (alt+.dz) in

    let new_pos = ecef_distance current_ecef new_ecef > 2. in
    match moved, new_pos with
      | None, _ ->
          self#move dx dy;
          alt <- alt+.dz;
          if update then updated ()
      | Some _, true -> ()
      | Some _, false -> self#reset_moved ()
  method set_ground_alt ga = ground_alt <- ga
  method delete () =
    deleted <- true; (* BOF *)
    geomap#delete_georef name;
    wpt_group#destroy ()
  method zoom (z:float) =
    if List.length wpt_group#get_items > 0 then
      let a = wpt_group#i2w_affine in
      a.(0) <- 1./.z; a.(3) <- 1./.z;
      wpt_group#affine_absolute a
  initializer wpt_group#raise_to_top ()
  initializer self#zoom geomap#zoom_adj#value
  initializer ignore(geomap#zoom_adj#connect#value_changed (fun () -> self#zoom geomap#zoom_adj#value))
end

let gensym = let n = ref 0 in fun prefix -> incr n; prefix ^ string_of_int !n

let waypoint = fun ?show group ?(name = gensym "wp") ?alt en ->
  new waypoint ?show group name ?alt en

