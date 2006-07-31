(*
* $Id$
*
* Copyright (C) 2006 ENAC, Pierre-Sélim Huard, Pascal Brisset, Antoine Drouin
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
  val active = Hashtbl.create 5
  method add text = 
    if not (Hashtbl.mem active text) then begin
      view#buffer#insert text;
      view#buffer#insert "\n";
      Hashtbl.add active text ()
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
  let table = GPack.table
      ~rows: 1
      ~columns: 3
      ~row_spacings: 5
      ~col_spacings: 40
      ~packing: widget#add
      () in
  let update_color = fun flags_eb flags ->
    let color = if flags land 0x01 = 1 then "green" else "red" in
    flags_eb#coerce#misc#modify_bg [`NORMAL, `NAME color] in
object
  val parent = widget
  val table = table
  val mutable active_cno = []
  val mutable active_flags = []

  method svsinfo svid cno flags =
    if not (List.mem_assoc svid active_cno) then
      let rows = table#rows in
      let _svid_label = GMisc.label 
	  ~text: ("sat "^ svid) ~packing: (table#attach ~top: rows ~left: 0) () in
      let cno_label = GMisc.label 
	  ~text:(cno^" dB Hz") ~packing: (table#attach ~top: rows ~left: 1) () in
      let flags_eb = GBin.event_box ~width: 20 ~packing:(table#attach ~top: rows ~left: 2) ()
      in
      update_color flags_eb flags;
      active_cno <- (svid, cno_label)::active_cno;
      active_flags <- (svid, flags_eb)::active_flags;
      table#set_rows (table#rows +1)
    else if visible widget then
      let cno_label = List.assoc svid active_cno in
      let flags_eb = List.assoc svid active_flags in
      cno_label#set_label (cno^" dB Hz");
      update_color flags_eb flags
end

(*****************************************************************************)
(* pfd page                                                                  *)
(*****************************************************************************)
class pfd ?(visible = fun _ -> true) (widget: GBin.frame) =
  let horizon = new Horizon.h ~packing: widget#add 150 in
  let _lazy = fun f x -> if visible widget then f x in
    
object
  method set_attitude roll pitch =
    _lazy (horizon#set_attitude ((Deg>>Rad)roll)) ((Deg>>Rad)pitch)
  method set_alt (a:float) = _lazy horizon#set_alt a
  method set_climb (_c:float) = ()
  method set_speed (c:float) = _lazy horizon#set_speed c
end


(*****************************************************************************)
(* Misc page                                                                 *)
(*****************************************************************************)
class misc ~packing (widget: GBin.frame) =
  let table = GPack.table
      ~rows: 2
      ~columns: 2
      ~row_spacings: 5
      ~col_spacings: 40
      ~packing
      () in
  let label = fun text i j ->GMisc.label ~text ~packing:(table#attach ~top:i ~left:j) () in
  let _init =
    ignore (label "Wind speed" 0 0);
    ignore (label "Wind direction" 1 0) in
  let wind_speed = label "" 0 1
  and wind_dir = label "" 1 1 in
  object
    method set_wind_speed s = wind_speed#set_text s
    method set_wind_dir s = wind_dir#set_text s
  end

(*****************************************************************************)
(* Dataling settings paged                                                   *)
(*****************************************************************************)
let one_setting = fun i do_change packing s (tooltips:GData.tooltips) ->
  let f = fun a -> float_of_string (ExtXml.attrib s a) in
  let lower = f "min"
  and upper = f "max"
  and step_incr = f "step" in
  
  let hbox = GPack.hbox ~packing () in
  let text = ExtXml.attrib s "var" in
  let _l = GMisc.label ~width:100 ~text ~packing:hbox#pack () in
  let _v = GMisc.label ~width:50 ~text:"N/A" ~packing:hbox#pack () in
  (** For a small number of values, radio buttons *)
  let _n = truncate ((upper -. lower) /. step_incr) in
  let commit =
    if step_incr = 1. && upper -. lower <= 2. then
      let ilower = truncate lower
      and iupper = truncate upper in
      let label = Printf.sprintf "%d" ilower in
      let first = GButton.radio_button ~label ~packing:hbox#add () in
      let value = ref lower in
      ignore (first#connect#clicked (fun () -> value := lower));
      let group = first#group in
      for j = ilower+1 to iupper do
	let label = Printf.sprintf "%d" j in
	let b = GButton.radio_button ~group ~label ~packing:hbox#add () in
	ignore (b#connect#clicked (fun () -> value := float j))
      done;
      (fun _ -> do_change i !value)
    else (* slider *)
      let value = (lower +. upper) /. 2. in
      let adj = GData.adjustment ~value ~lower ~upper:(upper+.10.) ~step_incr () in
      let _scale = GRange.scale `HORIZONTAL ~digits:2 ~adjustment:adj ~packing:hbox#add () in
      
      (fun _ -> do_change i adj#value)
  in
  let prev_value = ref None in
  let commit_but = GButton.button ~packing:hbox#pack () in
  commit_but#set_border_width 2;
  let _icon = GMisc.image ~stock:`APPLY ~packing:commit_but#add () in
  let callback = fun x ->
    prev_value := Some (float_of_string _v#text);
    commit x
  in
  ignore (commit_but#connect#clicked ~callback);
  tooltips#set_tip commit_but#coerce ~text:"Commit";
  let undo_but = GButton.button ~packing:hbox#pack () in
  let _icon = GMisc.image ~stock:`UNDO ~packing:undo_but#add () in
  let callback = fun _ ->
    match !prev_value with
      None -> ()
    | Some v -> do_change i v in
  ignore (undo_but#connect#clicked ~callback);
  tooltips#set_tip undo_but#coerce ~text:"Undo";
  _v  
  
  
let rec build_settings = fun do_change i flat_list xml_settings packing tooltips ->
  match xml_settings with
    [] -> ()
  | x::xs ->
      List.iter (fun y -> assert(ExtXml.tag_is y (Xml.tag x))) xs;
      if ExtXml.tag_is x "dl_setting" then
	List.iter
	  (fun s ->
	    let label_value = one_setting !i do_change packing s tooltips in
	    flat_list := label_value :: !flat_list;
	    incr i)
	  xml_settings
      else begin
	assert (ExtXml.tag_is x "dl_settings");
	let n = GPack.notebook ~packing () in
	
	List.iter (fun p ->
	  let text = ExtXml.attrib p "name" in
	  let _sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
	  let vbox = GPack.vbox  () in
	  
	  let tab_label = (GMisc.label ~text ())#coerce in
	  n#append_page ~tab_label vbox#coerce;

	  build_settings do_change i flat_list (Xml.children p) vbox#pack tooltips)
	  xml_settings
      end
  
  


class settings = fun ?(visible = fun _ -> true) xml_settings do_change ->
  let sw = GBin.scrolled_window ~hpolicy:`AUTOMATIC ~vpolicy:`AUTOMATIC () in
  let vbox = GPack.vbox ~packing:sw#add_with_viewport () in
  let tooltips = GData.tooltips () in
  let i = ref 0 and l = ref [] in
  let current_values =
    build_settings do_change i l xml_settings vbox#add tooltips;
    Array.of_list (List.rev !l) in
  object (self)
    method widget = sw#coerce
    method length = !i
    method set = fun i v ->
      if visible self#widget then
	let s = string_of_float v in
	if current_values.(i)#text <> s then
	  current_values.(i)#set_text s
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
  | x -> failwith (sprintf "Unknown rc_setting mode: %s" x)

let rc_setting_mode_index = function
    "UP" -> 0 | "DOWN" -> 1
  | x -> failwith (sprintf "Unknown rc_setting submode: %s" x)

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
	(* May raise an exception for non valid modes *)

	let s1 = string_of_float v1 in
	let s2 = string_of_float v2 in
	
	values.(i).(j).(0)#set_text s1;
	values.(i).(j).(1)#set_text s2
  end

