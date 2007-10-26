(*
 * $Id$
 *
 * Strip handling
 *  
 * Copyright (C) 2006 ENAC, Pascal Brisset, Antoine Drouin
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
module LL=Latlong

let (//) = Filename.concat

type t =
    < add_widget : GObj.widget -> unit;
      connect_shift_alt : (float -> unit) -> unit;
      connect_shift_lateral : (float -> unit) -> unit;
      connect_launch : (float -> unit) -> unit;
      connect_kill : (float -> unit) -> unit;
      connect_mode : (float -> unit) -> unit;
      connect_flight_time : (float -> unit) -> unit;
      set_agl : float -> unit;
      set_bat : float -> unit;
      set_throttle : float -> unit;
      set_speed : float -> unit;
      set_airspeed : float -> unit;
      set_climb : float -> unit;
      set_color : string -> string -> unit;
      set_label : string -> string -> unit;
      connect : (unit -> unit) -> unit; 
      hide_buttons : unit -> unit; 
      show_buttons : unit -> unit >

let bat_max = 12.5
let bat_min = 9.
let agl_max = 150.

(** window for the strip panel *)
let scrolled = GBin.scrolled_window ~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC ()
let strips_table = GPack.vbox ~spacing:5 ~packing:scrolled#add_with_viewport ()




(** set a label *)
let set_label labels name value = 
  try
    let _eb, l = List.assoc (name^"_value") labels in
    let value = sprintf "<b>%s</b>" value in
    if l#text <> value then
      l#set_label value
  with
    Not_found ->
      fprintf stderr "Strip.set_label: '%s' unknown\n%!" name

(** set a color *)
let set_color labels name color = 
  let eb, _l = List.assoc (name^"_value") labels in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color]


class gauge = fun ?(color="green") ?(history_len=50) gauge v_min v_max ->
  object
    val history = Array.create history_len 0
    val mutable history_index = -1
    method set = fun ?arrow value strings ->
      let {Gtk.width=width; height=height} = gauge#misc#allocation in
      if height > 1 then (* Else the drawing area is not allocated already *)
	let dr = GDraw.pixmap ~width ~height ~window:gauge () in
	dr#set_foreground (`NAME "orange");
	dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
	
	let f = (value -. v_min) /. (v_max -. v_min) in
	let f = max 0. (min 1. f) in
	let h = truncate (float height *. f) in
	
	(* First call: fill the array with the given value *)
	if history_index < 0 then begin
	  for i = 0 to history_len - 1 do
	    history.(i) <- h
	  done;
	  history_index <- 0;
	end;
	
	(* Store the value in the history array and update index *)
	history.(history_index) <- h;
	history_index <- (history_index+1) mod history_len;
	
	dr#set_foreground (`NAME color);
	
	(* From left to right, older to new values *)
	let polygon = ref [0,height; width,height] in
	for i = 0 to history_len - 1 do
	  let idx = (history_index+i) mod history_len in
	  polygon := ((i*width)/history_len, (height-history.(idx))):: !polygon;
	done;
	polygon := (width,height-h):: !polygon;
	dr#polygon ~filled:true !polygon;

	(* Arrow for the variation *)
	begin
	  match arrow with
	    None -> ()
	  | Some angle_rad ->
	      let w = width and h = height in
	      let fh = 0.8 *. float w in
	      let x = truncate (cos angle_rad *. fh)
	      and y = - truncate (sin angle_rad *. fh) in
	      let a = -.angle_rad +. 5. *. LL.pi /. 6.
	      and a' = -.angle_rad -. 5. *. LL.pi /. 6.
	      and al = 0.2 *. fh in
	      let ax = truncate (cos a *. al)
	      and ay = truncate (sin a *. al) in
	      let ax' = truncate (cos a' *. al)
	      and ay' = truncate (sin a' *. al) in
	      let l = [w/10, h/2; w/10+x,h/2+y; w/10+x+ax,h/2+y+ay; w/10+x,h/2+y; w/10+x+ax',h/2+y+ay'] in
	      dr#set_foreground `BLACK;
	      dr#lines l	      
	end;
	
	let context = gauge#misc#create_pango_context in
	List.iter (fun (vpos, string) ->
	  let layout = context#create_layout in
	  let fd = Pango.Context.get_font_description (Pango.Layout.get_context layout) in
	  Pango.Font.modify fd ~weight:`BOLD ();
	  context#set_font_description fd;
	  Pango.Layout.set_text layout string;
	  let (w,h) = Pango.Layout.get_pixel_size layout in
	  let y = truncate (vpos *. float height) - h / 2 in
	  dr#put_layout ~x:((width-w)/2) ~y ~fore:`BLACK layout)
	  strings;
	
	(new GDraw.drawable gauge#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
  end

class hgauge = fun ?(color="green") gauge v_min v_max ->
  object
    method set = fun value string ->
      let {Gtk.width=width; height=height} = gauge#misc#allocation in
      if height > 1 then (* Else the drawing area is not allocated already *)
	let dr = GDraw.pixmap ~width ~height ~window:gauge () in
	dr#set_foreground (`NAME "orange");
	dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
	
	let f = (value -. v_min) /. (v_max -. v_min) in
	let f = max 0. (min 1. f) in
	let w = truncate (float width *. f) in

	dr#set_foreground (`NAME color);
	dr#rectangle ~x:0 ~y:0 ~width:w ~height ~filled:true ();
	
	let context = gauge#misc#create_pango_context in
	let layout = context#create_layout in
	let fd = Pango.Context.get_font_description (Pango.Layout.get_context layout) in
	Pango.Font.modify fd ~weight:`BOLD ();
	context#set_font_description fd;
	Pango.Layout.set_text layout string;
	let (w,h) = Pango.Layout.get_pixel_size layout in
	dr#put_layout ~x:((width-w)/2) ~y:((height-h)/2) ~fore:`BLACK layout;
	
	(new GDraw.drawable gauge#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
  end

(** add a strip to the panel *)
let add = fun config color center_ac mark ->
  let strip_labels = ref  [] in
  let add_label = fun name value -> 
    strip_labels := (name, value) :: !strip_labels in

  let ac_name = Pprz.string_assoc "ac_name" config in

  let file = Env.paparazzi_src // "sw" // "ground_segment" // "cockpit" // "gcs.glade" in
  let strip = new Gtk_gcs.eventbox_strip ~file () in

  let eventbox_dummy = GBin.event_box () in

  strips_table#pack strip#toplevel#coerce;

  (* Name in top left *)
  strip#label_ac_name#set_label (sprintf "<b>%s</b>" ac_name);

  (* Color *)
  let plane_color = strip#eventbox_strip in
  plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];

  add_label "flight_time_value" (eventbox_dummy, strip#label_flight_time);
  add_label "block_time_value" (eventbox_dummy, strip#label_block_time);
  add_label "stage_time_value" (eventbox_dummy, strip#label_stage_time);
  add_label "block_name_value" (eventbox_dummy, strip#label_block_name);

  (* battery gauge *)
  let bat_da = strip#drawingarea_battery in
  bat_da#misc#realize ();
  let bat = new gauge bat_da bat_min bat_max in

  (* AGL gauge *)
  let agl_da = strip#drawingarea_agl in
  agl_da#misc#realize ();
  let agl = new gauge agl_da 0. agl_max in

  (* Speed gauge *)
  strip#drawingarea_speed#misc#realize ();
  let speed = new hgauge strip#drawingarea_speed 0. 10. in (* FIXME *)

  (* Throttle gauge *)
  strip#drawingarea_throttle#misc#realize ();
  let throttle = new hgauge strip#drawingarea_throttle 0. 100. in

  (* Diff to target altitude *)
  let diff_target_alt = strip#label_diff_target_alt in
  add_label "diff_target_alt_value" (eventbox_dummy, diff_target_alt);

  (* Telemetry *)
  let eb = strip#eventbox_telemetry in
  let ts = strip#label_telemetry in
  add_label "telemetry_status_value" (eb, ts);
  ts#set_width_chars 3;

  (* Labels *)
  add_label "RC_value" (strip#eventbox_rc, strip#label_rc);
  add_label "AP_value" (strip#eventbox_mode, strip#label_mode);
  add_label "GPS_value" (strip#eventbox_gps, strip#label_gps);

  add_label "altitude_value" (eventbox_dummy, strip#label_altitude);
  add_label "target_altitude_value" (eventbox_dummy, strip#label_target_altitude);

  add_label "eta_time_value" (eventbox_dummy, strip#label_eta_time);

  let connect_buttons = fun callback ->
    List.iter (fun ((button:GButton.button), value) ->
      ignore (button#connect#clicked (fun () -> callback value));
      button#misc#set_sensitive true) in

  (* Buttons : setting the icons (the path of the icon is not saved by glade) *)
  List.iter (fun (b, icon) ->
    b#remove b#child;
    let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // icon) in
    ignore (GMisc.image ~pixbuf ~packing:b#add ()))
    [ strip#button_launch, "launch.png";
      strip#button_kill, "kill.png";
      strip#button_resurrect, "resurrect.png";
      strip#button_minus_five, "down.png";
      strip#button_plus_five, "up.png";
      strip#button_plus_thirty, "upup.png";
      strip#button_left, "left.png";
      strip#button_center, "recenter.png";
      strip#button_right, "right.png";
    ];

   object
    val mutable climb = 0.
    method set_climb = fun v -> climb <- v
    method set_agl value = 
      let arrow = max (min 0.5 (climb /. 5.)) (-0.5) in
      agl#set ~arrow value [0.2, (sprintf "%3.0f" value); 0.8, sprintf "%+.1f" climb]
    method set_bat value = bat#set value [0.5, (string_of_float value)]
    method set_throttle value = throttle#set value (sprintf "%.0f%%" value)
    method set_speed value = speed#set value (sprintf "%.1fm/s" value)

    method set_airspeed value =
      let text = sprintf "Ground speed (est. airspeed: %.1fm/s)" value in
      let tooltips = GData.tooltips () in
      tooltips#set_tip strip#eventbox_speed#coerce ~text

    method set_label name value = set_label !strip_labels name value
    method set_color name value = set_color !strip_labels name value
    method add_widget w = strip#hbox_user#pack ~fill:false w
    method connect_shift_alt callback = 
      connect_buttons callback
	[ strip#button_minus_five, -5.;
 	  strip#button_plus_five, 5.;
	  strip#button_plus_thirty, 30.]
	
    method connect_shift_lateral = fun callback ->
      connect_buttons callback
	[ strip#button_left, -5.;
	  strip#button_right, 5.;
	  strip#button_center, 0.]
	
    method connect_kill = fun callback ->
      let callback = fun x ->
	if x = 1. then
	  match GToolbox.question_box ~title:"Kill throttle" ~buttons:["Kill"; "Cancel"] (sprintf "Kill throttle of A/C %s ?" ac_name) with
	    1 -> callback 1.
	  | _ -> ()
	else (* No confirmation for resurrect *)
	  callback x
      in
      connect_buttons callback
	[ strip#button_kill, 1.;
	  strip#button_resurrect, 0.]
	
    method connect_launch = fun callback ->
      connect_buttons callback
	[ strip#button_launch, 1. ]

    method connect_mode = fun callback ->
      let callback = fun _ -> (* Back in AUTO2 *)
	match GToolbox.question_box ~title:"Back to auto" ~buttons:["AUTO"; "Cancel"] (sprintf "Restore AUTO mode for A/C %s ?" ac_name) with
	  1 -> callback 2.; true
	| _ -> true in
      ignore(strip#eventbox_mode#event#connect#button_press ~callback)
	
    (* Reset the flight time *)
    method connect_flight_time = fun callback ->
      let callback = fun _ -> (* Reset flight time *)
	match GToolbox.question_box ~title:"Reset flight time" ~buttons:["Reset"; "Cancel"] (sprintf "Reset flight time for A/C %s ?" ac_name) with
	  1 -> callback 0.; true
	| _ -> true in
      ignore(strip#eventbox_flight_time#event#connect#button_press ~callback)
	 
    method hide_buttons () = strip#hbox_user#misc#hide (); strip#frame_nav#misc#set_sensitive false
    method show_buttons () = strip#hbox_user#misc#show (); strip#frame_nav#misc#set_sensitive true
    method connect = fun (select: unit -> unit) ->
      let callback = fun _ -> select (); true in
      ignore (strip#eventbox_strip#event#connect#button_press ~callback)
  end
