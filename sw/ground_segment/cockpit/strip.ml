(*
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
    < add_widget : ?group:string -> GObj.widget -> unit;
  connect_shift_alt : (float -> unit) -> unit;
  connect_shift_lateral : (float -> unit) -> unit;
  connect_launch : (float -> unit) -> unit;
  connect_kill : bool -> (float -> unit) -> unit;
  connect_mode : float -> (float -> unit) -> unit;
  connect_mark : (unit -> unit) -> unit;
  connect_flight_time : (float -> unit) -> unit;
  connect_apt : (unit -> float) -> (float -> unit) -> unit;
  set_agl : float -> unit;
  set_bat : float -> unit;
  set_throttle : ?kill:bool -> float -> unit;
  set_speed : float -> unit;
  set_airspeed : float -> unit;
  set_climb : float -> unit;
  set_color : string -> string -> unit;
  set_label : string -> string -> unit;
  set_rc : float -> string -> unit;
  connect : (unit -> unit) -> unit;
  hide_buttons : unit -> unit;
  show_buttons : unit -> unit >

type strip_param = {
  color : string;
  min_bat : float;
  max_bat : float;
  nb_cell_bat : float option;
  alt_shift_plus_plus : float;
  alt_shift_plus : float;
  alt_shift_minus : float;
  icons_theme : string }


let agl_max = 150.

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

class gauge = fun (gauge_da:GMisc.drawing_area) ->
object (self)
  inherit Gtk_tools.pixmap_in_drawin_area ~drawing_area:gauge_da ()
  method layout = fun string ->
    let context = gauge_da#misc#create_pango_context in
    let layout = context#create_layout in
    let fd = Pango.Context.get_font_description (Pango.Layout.get_context layout) in
    Pango.Font.modify fd ~weight:`BOLD ();
    context#set_font_description fd;
    Pango.Layout.set_text layout string;
    layout
  method request_width = fun string ->
    let layout = self#layout string in
    let (width,_h) = Pango.Layout.get_pixel_size layout in
    (gauge_da#misc#set_size_request ~width () : unit)
end

(* since tcl8.6 "green" refers to "darkgreen" and the former "green" is now "lime", but that is not available in older versions, so hardcode the color to #00ff00 *)
class vgauge = fun ?(color="#00ff00") ?(history_len=50) gauge_da v_min v_max ->
object (self)
  inherit gauge gauge_da
  val history = Array.make history_len 0
  val mutable history_index = -1
  method set = fun ?arrow ?(background="orange") value strings ->
    let {Gtk.width=width; height=height} = gauge_da#misc#allocation in
    if height > 1 then (* Else the drawing area is not allocated already *)
      let dr = self#get_pixmap () in
      dr#set_foreground (`NAME background);
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

      List.iter (fun (vpos, string) ->
        let layout = self#layout string in
        let (w,h) = Pango.Layout.get_pixel_size layout in
        let y = truncate (vpos *. float height) - h / 2 in
        dr#put_layout ~x:((width-w)/2) ~y ~fore:`BLACK layout)
        strings;

      (new GDraw.drawable gauge_da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
end

class hgauge = fun ?(color="#00ff00") gauge_da v_min v_max ->
object (self)
  inherit gauge gauge_da
  method set = fun ?(background="orange") value string ->
    let {Gtk.width=width; height=height} = gauge_da#misc#allocation in
    if height > 1 then (* Else the drawing area is not allocated already *)
      let dr = self#get_pixmap () in
      dr#set_foreground (`NAME background);
      dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

      let f = (value -. v_min) /. (v_max -. v_min) in
      let f = max 0. (min 1. f) in
      let w = truncate (float width *. f) in

      dr#set_foreground (`NAME color);
      dr#rectangle ~x:0 ~y:0 ~width:w ~height ~filled:true ();

      let layout = self#layout string in
      let (w,h) = Pango.Layout.get_pixel_size layout in
      dr#put_layout ~x:((width-w)/2) ~y:((height-h)/2) ~fore:`BLACK layout;

      (new GDraw.drawable gauge_da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
end

(** add a strip to the panel *)
(*let add = fun config color min_bat max_bat ->*)
let add = fun config strip_param (strips:GPack.box) ->
  let color = strip_param.color
  and min_bat = strip_param.min_bat
  and max_bat = strip_param.max_bat
  and nb_cell_bat = strip_param.nb_cell_bat
  and alt_shift_plus_plus = strip_param.alt_shift_plus_plus
  and alt_shift_plus = strip_param.alt_shift_plus
  and alt_shift_minus = strip_param.alt_shift_minus in

  let strip_labels = ref  [] in
  let add_label = fun name value ->
    strip_labels := (name, value) :: !strip_labels in

  let ac_name = PprzLink.string_assoc "ac_name" config in

  let file = Env.paparazzi_src // "sw" // "ground_segment" // "cockpit" // "gcs.glade" in
  let strip = new Gtk_strip.eventbox_strip ~file () in

  let eventbox_dummy = GBin.event_box () in

  strips#pack strip#toplevel#coerce;

  (* Name in top left *)
  strip#label_ac_name#set_label (sprintf "<b>%s</b>" ac_name);

  (* Color *)
  let plane_color = strip#eventbox_strip in
  plane_color#coerce#misc#modify_bg [`NORMAL, `NAME color];

  add_label "flight_time_value" (eventbox_dummy, strip#label_flight_time);
  add_label "block_time_value" (eventbox_dummy, strip#label_block_time);
  add_label "stage_time_value" (eventbox_dummy, strip#label_stage_time);
  add_label "block_name_value" (eventbox_dummy, strip#label_block_name);
  add_label "apt_value" (eventbox_dummy, strip#label_apt_value);

  (* battery gauge *)
  let bat_da = strip#drawingarea_battery in
  bat_da#misc#realize ();
  let bat = new vgauge bat_da min_bat max_bat in
  bat#request_width "22.5";
  bat#set 0. [0.5, "UNK"];

  (* AGL gauge *)
  let agl_da = strip#drawingarea_agl in
  agl_da#misc#realize ();
  let agl = new vgauge agl_da 0. agl_max in

  (* Speed gauge *)
  strip#drawingarea_speed#misc#realize ();
  let speed = new hgauge strip#drawingarea_speed 0. 10. in (* FIXME *)
  speed#request_width "33.3m/s";

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

  (* RC *)
  strip#drawingarea_rc#misc#realize ();
  let rc = new hgauge strip#drawingarea_rc 0. 10. in
  rc#request_width "NONE";

  (* Labels *)
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
    try
      let pixbuf = GdkPixbuf.from_file (Env.get_gcs_icon_path strip_param.icons_theme icon) in
      ignore (GMisc.image ~pixbuf ~packing:b#add ())
    with
        exc ->
          fprintf stderr "Error: %s\n" (Printexc.to_string exc);
          ignore (GMisc.label ~text:"?" ~packing:b#add ()))
    [ strip#button_launch, "launch.png";
      strip#button_kill, "kill.png";
      strip#button_resurrect, "resurrect.png";
      strip#button_down, "down.png";
      strip#button_up, "up.png";
      strip#button_up_up, "upup.png";
      strip#button_left, "left.png";
      strip#button_center, "recenter.png";
      strip#button_right, "right.png";
    ];

object
  val mutable climb = 0.
  val mutable button_tbl = Hashtbl.create 10
  method set_climb = fun v -> climb <- v
  method set_agl value =
    let arrow = max (min 0.5 (climb /. 5.)) (-0.5) in
    agl#set ~arrow value [0.2, (sprintf "%3.0f" value); 0.8, sprintf "%+.1f" climb]
  method set_bat value =
    let v = if value < 0.1 then "UNK" else (string_of_float value) in
    match nb_cell_bat with
    | None -> bat#set value [0.5, v]
    | Some nb -> bat#set value [0.3, v; 0.7, sprintf "%.2f /c" (value /. nb)]
  method set_throttle ?(kill=false) value =
    let background = if kill then "red" else "orange" in
    throttle#set ~background value (sprintf "%.0f%%" value)
  method set_speed value = speed#set value (sprintf "%.1fm/s" value)

  method set_airspeed value =
    let text = sprintf "Ground speed (est. airspeed: %.1fm/s)" value in
    let tooltips = GData.tooltips () in
    tooltips#set_tip strip#eventbox_speed#coerce ~text

  method connect_mark callback =
    ignore (strip#button_mark#connect#clicked callback)

  method set_label name value = set_label !strip_labels name value
  method set_color name value = set_color !strip_labels name value

  method set_rc rate status = rc#set rate status

    (* add a button widget in a vertical box if it belongs to a group (create new group if needed) *)
  method add_widget ?(group="") w =
    let (vbox, pack) = match String.length group with
        0 -> (GPack.vbox ~show:true (), true)
      | _ -> try (Hashtbl.find button_tbl group, false) with
          Not_found ->
            let vb = GPack.vbox ~show:true () in
            ignore(Hashtbl.add button_tbl group vb);
            (vb, true)
    in
      (*let vbox = GPack.vbox ~show:true () in*)
    vbox#pack ~fill:false w;
    if pack then strip#hbox_user#pack ~fill:false vbox#coerce else ()

  method connect_shift_alt callback =
    let tooltips = GData.tooltips () in
    let text = Printf.sprintf "Altitude %+.1fm" alt_shift_minus in
    ignore (tooltips#set_tip strip#button_down#coerce ~text);
    let text = Printf.sprintf "Altitude %+.1fm" alt_shift_plus in
    ignore (tooltips#set_tip strip#button_up#coerce ~text);
    let text = Printf.sprintf "Altitude %+.1fm" alt_shift_plus_plus in
    ignore (tooltips#set_tip strip#button_up_up#coerce ~text);
    connect_buttons callback
      [ strip#button_down, alt_shift_minus;
        strip#button_up, alt_shift_plus;
        strip#button_up_up, alt_shift_plus_plus]

  method connect_shift_lateral = fun callback ->
    connect_buttons callback
      [ strip#button_left, -5.;
        strip#button_right, 5.;
        strip#button_center, 0.]

  method connect_kill = fun confirm_kill callback ->
    let callback = fun x ->
      if x = 1. && confirm_kill then
        match GToolbox.question_box ~title:"Kill throttle" ~buttons:["Kill"; "Cancel"] (sprintf "Kill throttle of A/C %s ?" ac_name) with
            1 -> callback 1.
          | _ -> ()
      else (* No confirmation for resurrect or confirm_kill = false *)
        callback x
    in
    connect_buttons callback
      [ strip#button_kill, 1.;
        strip#button_resurrect, 0.]

  method connect_launch = fun callback ->
    connect_buttons callback
      [ strip#button_launch, 1. ]

  method connect_mode = fun mode callback ->
    let callback = fun _ -> (* Back in AUTO2 *)
      match GToolbox.question_box ~title:"Back to auto" ~buttons:["AUTO"; "Cancel"] (sprintf "Restore AUTO mode for A/C %s ?" ac_name) with
          1 -> callback mode; true
        | _ -> true in
    ignore(strip#eventbox_mode#event#connect#button_press ~callback)

    (* Reset the flight time *)
  method connect_flight_time = fun callback ->
    let callback = fun _ -> (* Reset flight time *)
      match GToolbox.question_box ~title:"Reset flight time" ~buttons:["Reset"; "Cancel"] (sprintf "Reset flight time for A/C %s ?" ac_name) with
          1 -> callback 0.; true
        | _ -> true in
    ignore(strip#eventbox_flight_time#event#connect#button_press ~callback)

     (** Appointment date *)
  method connect_apt = fun get_ac_unix_time send_value ->
    strip#label_apt#misc#show ();
    strip#label_apt_value#misc#show ();
    let callback = fun _ ->
      let w = new Gtk_setting_time.setting_time ~file () in
      let utc = Unix.gmtime (get_ac_unix_time () +. 60.) in
      w#spinbutton_hour#set_value (float utc.Unix.tm_hour);
      w#spinbutton_min#set_value (float utc.Unix.tm_min);
      ignore (w#button_cancel#connect#clicked (fun () -> w#setting_time#destroy ()));
      let callback = fun () ->
        let hour = truncate w#spinbutton_hour#value
        and min = truncate w#spinbutton_min#value
        and sec = truncate w#spinbutton_sec#value in
        w#setting_time#destroy ();
        let tow = Latlong.gps_tow_of_utc hour min sec in
        send_value (float tow) in
      ignore (w#button_ok#connect#clicked callback);
      true
    in
    ignore(strip#eventbox_RDV#event#connect#button_press ~callback)


  method hide_buttons () = strip#hbox_user#misc#hide (); strip#frame_nav#misc#set_sensitive false
  method show_buttons () = strip#hbox_user#misc#show (); strip#frame_nav#misc#set_sensitive true
  method connect = fun (select: unit -> unit) ->
    let callback = fun _ -> select (); true in
    ignore (strip#eventbox_strip#event#connect#button_press ~callback)
end
