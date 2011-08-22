(*
 * $Id$
 *
 * Copyright (C) 2006 ENAC, Pierre-S�lim Huard, Pascal Brisset, Antoine Drouin
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

(*****************************************************************************)
(* Information pages such as alert, infrared, gps, artificial horizon        *)
(*****************************************************************************)

let (//) = Filename.concat

open Latlong
open Printf

(** alert page *)
class alert (widget: GBin.frame) =
  let scrolled = GBin.scrolled_window
      ~hpolicy: `AUTOMATIC
      ~vpolicy: `AUTOMATIC
      ~packing: widget#add
      ()
  in
  let view = GText.view ~editable:false ~packing: scrolled#add () in
(* the object itselft *)
  object
    val mutable last = ""
    method add text =
      if text <> last then begin
	let l = Unix.localtime (Unix.gettimeofday ()) in
	view#buffer#insert (sprintf "%02d:%02d:%02d " l.Unix.tm_hour l.Unix.tm_min l.Unix.tm_sec);
	view#buffer#insert text;
	view#buffer#insert "\n";

	(* Scroll to the bottom line *)
	let end_iter = view#buffer#end_iter in
	let end_mark = view#buffer#create_mark end_iter in
	view#scroll_mark_onscreen (`MARK end_mark);

	last <- text
      end
  end

(*****************************************************************************)
(* infrared page                                                             *)
(*****************************************************************************)
class infrared (widget: GBin.frame) =
  let table = GPack.table
      ~rows: 4
      ~columns: 2
      ~row_spacings: 5
      ~col_spacings: 5
      ~packing: widget#add
      ()
  in
  let contrast_status =
    GMisc.label ~text: "" ~packing: (table#attach ~top:0 ~left: 1) ()
  in
  let contrast_value =
    GMisc.label ~text: "" ~packing: (table#attach ~top:1 ~left: 1) ()
  in
  let gps_hybrid_mode =
    GMisc.label ~text: "" ~packing: (table#attach ~top:2 ~left: 1) ()
  in
  let gps_hybrid_factor =
    GMisc.label ~text: "" ~packing: (table#attach ~top:3 ~left: 1) ()
  in
  let _init =
    ignore (GMisc.label ~text: "contrast status" ~packing: (table#attach ~top:0 ~left: 0) ());
    ignore (GMisc.label ~text: "contrast" ~packing: (table#attach ~top:1 ~left: 0) ());
    ignore (GMisc.label ~text: "gps hybrid mode" ~packing: (table#attach ~top:2 ~left: 0) ());
    ignore (GMisc.label ~text: "gps hybrid factor" ~packing: (table#attach ~top:3 ~left: 0) ())
  in
  object
    val parent = widget
    val table = table

    val contrast_status = contrast_status
    val contrast_value = contrast_value
    val gps_hybrid_mode = gps_hybrid_mode
    val gps_hybrid_factor = gps_hybrid_factor

    method set_contrast_status (s:string) =
      contrast_status#set_label s
    method set_contrast_value (s:int) =
      contrast_value#set_label (Printf.sprintf "%d" s)
    method set_gps_hybrid_mode (s:string) =
      gps_hybrid_mode#set_label s
    method set_gps_hybrid_factor (s:float) =
      gps_hybrid_factor#set_label (Printf.sprintf "%.8f" s)
  end

(*****************************************************************************)
(* gps page                                                                  *)
(*****************************************************************************)
class gps ?(visible = fun _ -> true) (widget: GBin.frame) =
  let vbox = GPack.vbox ~packing:widget#add () in

  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC ~packing:vbox#add () in

  let da_object = new Gtk_tools.pixmap_in_drawin_area ~packing:sw#add_with_viewport () in

  (* Reset buttons *)
  let hbox = GPack.hbox ~packing:vbox#pack ~show:false () in
  let _ = GMisc.label ~text:"Reset: " ~packing:hbox#add () in
  let hot = GButton.button ~label:"Hostart" ~packing:hbox#add () in
  let warm = GButton.button ~label:"Warmstart" ~packing:hbox#add () in
  let cold = GButton.button ~label:"Coldstart" ~packing:hbox#add () in

  object
    val mutable active_cno = []
    val mutable active_flags = []

    method connect_reset = fun (callback:int -> unit) ->
      hbox#misc#show ();
      ignore (hot#connect#clicked (fun () -> callback 0));
      ignore (warm#connect#clicked (fun () -> callback 1));
      ignore (cold#connect#clicked (fun () -> callback 2))

    method svsinfo pacc a =
      if visible widget then
	let da = da_object#drawing_area in
	let {Gtk.width=width; height=height} = da#misc#allocation in

	(* Background *)
	let dr = da_object#get_pixmap () in
	dr#set_foreground (`NAME "white");
	dr#rectangle ~x:0 ~y:0 ~width ~height ~filled:true ();

	let context = da#misc#create_pango_context in
	context#set_font_by_name ("sans " ^ string_of_int 10);
	let layout = context#create_layout in

	let n = Array.length a in
	let sep_size = 3 in
	let indic_size = min 25 ((width-(n+1)*sep_size)/n) in
	let max_cn0 = 50 in

	Pango.Layout.set_text layout "Dummy";
	let (_, h) = Pango.Layout.get_pixel_size layout in

	let size = fun cn0 -> (max 20 cn0 - 20) * 2 in

	let y = sep_size + h + (size max_cn0) in
	for i = 0 to n - 1 do
	  let (id, cn0, flags, age) = a.(i) in
	  if age < 60 then
	    let x = sep_size + i * (sep_size+indic_size) in

	    (* level *)
	    Pango.Layout.set_text layout (sprintf "% 2d" cn0);
	    dr#put_layout ~x ~y:0 ~fore:`BLACK layout;

	    (* bar *)
	    let color = if age > 5 then "grey" else if flags land 0x01 = 1 then "green" else "red" in
	    dr#set_foreground (`NAME color);
	    let height = size cn0 in
	    dr#rectangle ~filled:true ~x ~y:(y-height) ~width:indic_size ~height ();
	    (* SV id *)
	    Pango.Layout.set_text layout (sprintf "% 2d" id);
	    dr#put_layout ~x ~y ~fore:`BLACK layout
	done;

	(* Pacc *)
	let max_pacc = 2000 in
	dr#set_foreground (`NAME "red");
	let w = min width ((pacc*width)/max_pacc) in
	dr#rectangle ~filled:true ~x:0 ~y:(y+h) ~width:w ~height:h ();
	Pango.Layout.set_text layout (if pacc = 0 then "Pos accuracy: N/A" else sprintf "Pos accuracy: %.1fm" (float pacc/.100.));
	let (_, h) = Pango.Layout.get_pixel_size layout in
	dr#put_layout ~x:((width-w)/2) ~y:(y+h) ~fore:`BLACK layout;

	(new GDraw.drawable da#misc#window)#put_pixmap ~x:0 ~y:0 dr#pixmap
  end

(*****************************************************************************)
(* Misc page                                                                 *)
(*****************************************************************************)
let misc_fields = [|"Wind speed"; "Wind direction"; "Mean airspeed"; "Time to HOME"; "Send periodically"|]
let index_of = fun label ->
  let rec search = fun i ->
    if i < Array.length misc_fields then
      if misc_fields.(i) = label then i else search (i+1)
    else
      failwith (sprintf "Unknown label in Misc.index_of: %s" label)
  in
  search 0

class misc ~packing (widget: GBin.frame) =
  let rows = Array.length misc_fields in
  let table = GPack.table ~rows ~columns:2 ~row_spacings:5 ~col_spacings:40 ~packing () in
  let label = fun text i j -> GMisc.label ~text ~packing:(table#attach ~top:i ~left:j) () in
  let values =
    Array.init rows (fun i ->
      ignore (label misc_fields.(i) i 0);
      label "N/A" i 1) in
  (* Overwrite the "Send periodically" value with a check box *)
  let periodic_send =
    let top = index_of "Send periodically" in
    values.(top)#destroy ();
    GButton.check_button ~active:true ~packing:(table#attach ~top ~left:1) () in
  object
    method set_value label s = values.(index_of label)#set_text s
    method periodic_send = periodic_send#active
  end

type rc_mode = string
type rc_setting_mode = string

let rc_setting_index = function
    "gain_1_up" -> 0, 0
  | "gain_1_down" -> 1, 0
  | "gain_2_up" -> 0, 1
  | "gain_2_down" -> 1, 1
  | x -> failwith (sprintf "Unknown rc_setting: %s" x)

let rc_mode_index = function
    "AUTO1" -> 0 | "AUTO2" -> 1
  | _x -> -1

let rc_setting_mode_index = function
    "UP" -> 0 | "DOWN" -> 1
  | _x -> -1

let one_rc_mode = fun (table:GPack.table) rc_mode ->
  let i = rc_mode_index (ExtXml.attrib rc_mode "name") in
  List.iter (fun rc_setting ->
    let name = ExtXml.attrib rc_setting "rc"
    and text = ExtXml.attrib rc_setting "var" in
    let (j, k) = rc_setting_index name in
    ignore (GMisc.label ~text ~packing:(table#attach ~top:(1+2*j+k) ~left:(1+2*i)) ())
	    )
    (Xml.children rc_mode)


class rc_settings = fun ?(visible = fun _ -> true) xmls ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let table = GPack.table ~rows:5 ~columns:5 ~row_spacings:8 ~packing:sw#add_with_viewport () in
  let auto1 = GBin.event_box ~packing:(table#attach ~top:0 ~left:1 ~right:3) () in
  let _ = GMisc.label ~text:"AUTO1" ~packing:auto1#add () in
  let auto2 = GBin.event_box ~packing:(table#attach ~top:0 ~left:3 ~right:5) () in
  let _ = GMisc.label ~text:"AUTO2" ~packing:auto2#add () in
  let up = GBin.event_box ~packing:(table#attach ~top:1 ~bottom:3 ~left:0) () in
  let _ = GMisc.label ~text:"UP" ~packing:up#add () in
  let down = GBin.event_box ~packing:(table#attach ~top:3 ~bottom:5 ~left:0) () in
  let _ = GMisc.label ~text:"DOWN" ~packing:down#add () in
  let update_bg = fun ev active ->
    ev#coerce#misc#modify_bg [`NORMAL, `NAME (if active then "green" else "white")] in
  (* first index is auto1/auto2, second is up/down, third is value 1/2 *)
  let values = Array.init 2 (fun i -> Array.init 2 (fun j -> Array.init 2 (fun k -> GMisc.label ~text:"  N/A  " ~packing:(table#attach ~top:(1+j*2+k) ~left:(2+i*2)) ()))) in

  let _ = List.iter (one_rc_mode table) xmls in

  object (self)
    val mutable rc_mode = "N/A"
    val mutable rc_setting_mode = "N/A"
    method set_rc_mode m =
      rc_mode <- m;
      update_bg auto1 (m="AUTO1");
      update_bg auto2 (m="AUTO2")

    method set_rc_setting_mode m =
      rc_setting_mode <- m;
      update_bg up (m="UP");
      update_bg down (m="DOWN")

    method widget = sw#coerce
    method set = fun v1 v2 ->
      if visible self#widget then
	let i = rc_mode_index rc_mode
	and j = rc_setting_mode_index rc_setting_mode in
	if i >= 0 && j >= 0 then
	  let s1 = string_of_float v1 in
	  let s2 = string_of_float v2 in

	  values.(i).(j).(0)#set_text s1;
	  values.(i).(j).(1)#set_text s2
  end

