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

type t =
    < add_widget : GObj.widget -> unit;
      connect_shift_alt : (float -> unit) -> unit;
	set_agl : ?color:string -> float -> unit;
	set_bat : ?color:string -> float -> unit;
	  set_color : string -> string -> unit;
	    set_label : string -> string -> unit >

let bat_max = 12.5
let bat_min = 9.
let agl_max = 150.

(** window for the strip panel *)
let scrolled = GBin.scrolled_window ~hpolicy: `AUTOMATIC ~vpolicy: `AUTOMATIC ()
let table = GPack.table ~rows: 1 ~columns: 1 ~row_spacings: 5 ~packing: (scrolled#add_with_viewport) ()




(** set a label *)
let set_label labels name value = 
  try
    let _eb, l = List.assoc (name^"_value") labels in
    if l#text <> value then
      l#set_label value
  with
    Not_found ->
      Printf.fprintf stderr "Strip.set_label: '%s' unknown\n%!" name

(** set a color *)
let set_color labels name color = 
  let eb, _l = List.assoc (name^"_value") labels in
  eb#coerce#misc#modify_bg [`NORMAL, `NAME color]


let set_gauge = fun ?(color="green") gauge v_min v_max value string ->
  let {Gtk.width=width; height=height} = gauge#misc#allocation in
  let dr = GDraw.pixmap ~width ~height ~window:gauge () in
  dr#set_foreground (`NAME "orange");
  dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();
  
  let f = (value -. v_min) /. (v_max -. v_min) in
  let f = max 0. (min 1. f) in
  let h = truncate (float height *. f) in
  dr#set_foreground (`NAME color);
  dr#rectangle ~x:0 ~y:(height-h) ~width ~height:h ~filled:true ();

  let context = gauge#misc#create_pango_context in
  let layout = context#create_layout in
  Pango.Layout.set_text layout string;
  let (w,h) = Pango.Layout.get_pixel_size layout in
  dr#put_layout ~x:((width-w)/2) ~y:((height-h)/2) ~fore:`BLACK layout;
  
  (new GDraw.drawable gauge#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap



(** set the battery *)
let set_bat ?color gauge value =
  set_gauge ?color gauge bat_min bat_max value (string_of_float value)


(** set the AGL *)
let set_agl ?color agl value =
  set_gauge ?color agl 0. agl_max value (Printf.sprintf "%3.0f" value)


let add_widget = fun buttons_box widget ->
  buttons_box#add widget

let labels_name =  [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "/" |]; [| "GPS"; "speed"; "throttle" |]
|]

let labels_print = [| 
  [| "AP" ; "alt" ; "->" |]; [| "RC"; "climb"; "->" |]; [| "GPS"; "speed"; "throttle" |]
|]
let gen_int = let i = ref (-1) in fun () -> incr i; !i

let rows = 1 + Array.length labels_name + 1
let columns = 1 + 2 * Array.length labels_name.(0) + 1






    (** add a strip to the panel *)
let add config color select center_ac mark =
  let widget = table in
  (* number of the strip *)
  let strip_number = gen_int () in
  if strip_number > 1 then widget#set_rows strip_number;

  let strip_labels = ref  [] in
  let add_label = fun name value -> 
    strip_labels := (name, value) :: !strip_labels in

  let ac_name = Pprz.string_assoc "ac_name" config in

  let tooltips = GData.tooltips () in

  (* frame of the strip *)
  let strip_ebox = GBin.event_box ~packing:(widget#attach ~top:strip_number ~left:0) () in 
  let frame = GBin.frame ~shadow_type: `IN ~packing:strip_ebox#add () in
  let framevb = GPack.vbox ~packing:frame#add () in

  (** Table (everything except the user buttons) *)
  let strip = GPack.table ~rows ~columns ~col_spacings:3 ~packing:framevb#add () in
  strip#set_row_spacing 0 3;
  strip#set_row_spacing (rows-2) 3;

  (* Name in top left *)
  let name = (GMisc.label ~text: (ac_name) ~packing: (strip#attach ~top: 0 ~left: 0) ()) in
  name#set_width_chars 5;
  
  
  let plane_color = GBin.event_box ~packing:(strip#attach ~top:0 ~left:1 ~right:columns) () in
  plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];
  ignore (strip_ebox#event#connect#button_press ~callback:(fun _ -> select (); true));
  let h = GPack.hbox ~packing:plane_color#add () in
  let ft = GMisc.label ~text: "00:00:00" ~packing:h#add () in
  ft#set_width_chars 8;
  add_label "flight_time_value" (plane_color, ft);

  let block_time = GMisc.label ~text: "00:00" ~packing:h#add () in
  add_label "block_time_value" (plane_color, block_time);

  let stage_time = GMisc.label ~text: "00:00" ~packing:h#add () in
  add_label "stage_time_value" (plane_color, stage_time);

  let block_name = GMisc.label ~text: "______" ~packing:h#add () in
  add_label "block_name_value" (plane_color, block_name);

  tooltips#set_tip plane_color#coerce ~text:"Flight time - Block time - Stage  time - Block name";

  (* battery gauge *)
  let gauge = GMisc.drawing_area ~height:60 ~show:true ~packing:(strip#attach ~top:1 ~bottom:(rows-1) ~left:0) () in
  gauge#misc#realize ();

  (* AGL gauge *)
  let agl_box = GBin.event_box ~packing:(strip#attach ~top:1 ~bottom:3 ~left:(columns-1)) () in
  let agl = GMisc.drawing_area ~width:30 ~height:60 ~show:true ~packing:agl_box#add () in
  agl#misc#realize ();
  tooltips#set_tip agl_box#coerce ~text:"AGL (m)";

  (* Diff to target altitude *)
  let dta_box = GBin.event_box ~packing:(strip#attach ~top:3 ~left:(columns-1)) () in
  let diff_target_alt = GMisc.label ~text: "+0" ~packing:dta_box#add () in
  add_label "diff_target_alt_value" (plane_color, diff_target_alt);
  tooltips#set_tip dta_box#coerce ~text:"Height to target (m)";

  (* Telemetry *)
  let eb = GBin.event_box ~packing:(strip#attach ~top:(rows-1) ~left:0) () in
  let ts = GMisc.label ~text:"N/A" ~packing:eb#add () in
  add_label "telemetry_status_value" (eb, ts);
  tooltips#set_tip eb#coerce ~text:"Telemetry status\nGreen if time since last bat message < 5s";

  (* Labels *)
  Array.iteri 
    (fun i a ->
      Array.iteri
	(fun j s ->
	  ignore (GMisc.label ~text: labels_print.(i).(j) ~justify:`RIGHT ~packing: (strip#attach ~top:(1+i) ~left: (1+2*j)) ());
	  let eb = GBin.event_box  ~packing: (strip#attach ~top:(i+1)  ~left: (1+2*j+1)) () in
	  let lvalue = (GMisc.label ~text: "" ~justify: `RIGHT ~packing:eb#add ()) in
	  lvalue#set_width_chars 6;
	  add_label (s^"_value") (eb, lvalue);
	) a
    ) labels_name;

  (* Buttons *)
  let top = rows - 1 in
  let b = GButton.button ~label:"Center A/C" ~packing:(strip#attach ~top ~left:1 ~right:3) () in
  ignore(b#connect#clicked ~callback:center_ac);
  let b = GButton.button ~label:"Mark" ~packing:(strip#attach ~top ~left:3 ~right:5) () in
  ignore (b#connect#clicked  ~callback:mark);

  let minus5 = GButton.button ~label:"-5m" ~packing:(strip#attach ~top ~left:5) ()
  and plus5 = GButton.button ~label:"+5m" ~packing:(strip#attach ~top ~left:6) ()
  and plus30 = GButton.button ~label:"+30m" ~packing:(strip#attach ~top ~left:7) () in
  ignore (b#connect#clicked  ~callback:mark);

  (* User buttons *)
  let hbox = GPack.hbox ~packing:framevb#add () in


  object
    method set_agl ?color value = set_agl ?color agl value
    method set_bat ?color value = set_bat ?color gauge value
    method set_label name value = set_label !strip_labels name value
    method set_color name value = set_color !strip_labels name value
    method add_widget w = add_widget hbox w
    method connect_shift_alt callback = 
      ignore (plus5#connect#clicked (fun () -> callback 5.));
      ignore (plus30#connect#clicked (fun () -> callback 30.));
      ignore (minus5#connect#clicked (fun () -> callback (-5.)))
  end
