(*
 * $Id$
 *
 * Handling papgets in the geomap canvas
 *
 * Copyright (C) 2008 ENAC
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

let papgets = Hashtbl.create 5
let register_papget = fun p p -> Hashtbl.add papgets p p
let dump_store = fun () ->
  Hashtbl.fold
    (fun _ p r ->
      if not p#deleted then
	p#config ()::r
      else
	r)
    papgets
    []

let papget_listener =
  let sep = Str.regexp ":" in
  fun papget ->
    try
      let field = Papget.get_property "field" papget in
      match Str.split sep field with
	[msg_name; field_name] ->
	  (new Papget.message msg_name, field_name)
      | _ -> failwith (sprintf "Unexpected field spec: %s" field)
    with
      _ -> failwith (sprintf "field attr expected in '%s" (Xml.to_string papget))
	  
let create = fun canvas_group papget ->
  let type_ = ExtXml.attrib papget "type"
  and display = ExtXml.attrib papget "display"
  and x = ExtXml.float_attrib papget "x"
  and y = ExtXml.float_attrib papget "y"
  and config = Xml.children papget in
  match type_ with
    "message_field" ->
      let msg_listener, field_name = papget_listener papget
      and renderer =
	match display with
	  "text" ->
	    (new Papget.canvas_text ~config canvas_group x y :> Papget.renderer)
	      
	| "ruler" ->
	    (new Papget.canvas_ruler canvas_group ~config x y :> Papget.renderer)
	| _ -> failwith (sprintf "Unexpected papget display: %s" display) in
      let p = new Papget.canvas_display_item msg_listener field_name renderer in
      let p = (p :> Papget.item) in
      register_papget p p
  | "goto_block" ->
(***
    let button = GButton.button ()
    and icon = Papget.get_property "icon" papget in
    let pixbuf = GdkPixbuf.from_file (Env.gcs_icons_path // icon) in
    ignore (GMisc.image ~pixbuf ~packing:button#add ());
    let renderer = (new Papget.widget_renderer "Button" button#coerce canvas_group ~config x y :> Papget.renderer) in
 ***)
      let renderer = 
	match display with
	  "button" ->
	    (new Papget.canvas_button canvas_group ~config x y :> Papget.renderer)
	| _ -> failwith (sprintf "Unexpected papget display: %s" display) in
      let block_name = Papget.get_property "block_name" papget in
      let clicked = fun () ->
	prerr_endline "Warning: goto_block papget sends to all A/C";
	Hashtbl.iter
	  (fun ac_id ac ->
	    let blocks = ExtXml.child ac.Live.fp "blocks" in
	    let block = ExtXml.child ~select:(fun x -> ExtXml.attrib x "name" = block_name) blocks "block" in
	    let block_id = ExtXml.int_attrib block "no" in
	    Live.jump_to_block ac_id block_id
	  )
	  Live.aircrafts
      in
      let properties = [ Papget.property "block_name" block_name ] in

      let p = new Papget.canvas_goto_block_item properties clicked renderer in
      let p = (p :> Papget.item) in
      register_papget p p
  | "variable_setting" ->
      let renderer = 
	match display with
	  "button" ->
	    (new Papget.canvas_button canvas_group ~config x y :> Papget.renderer)
	| _ -> failwith (sprintf "Unexpected papget display: %s" display) in

      let varname = Papget.get_property "variable" papget
      and value = float_of_string (Papget.get_property "value" papget) in

      let clicked = fun () ->
	prerr_endline "Warning: variable_setting papget sending to all active A/C";
	Hashtbl.iter
	  (fun ac_id ac ->
	    match ac.Live.dl_settings_page with
	      None -> ()
	    | Some settings ->
		let var_id = settings#assoc varname in
		Live.dl_setting ac_id var_id value)
	  Live.aircrafts
      in
      let properties = [ Papget.property "variable" varname;
			 Papget.float_property "value" value ] in
      let p = new Papget.canvas_variable_setting_item properties clicked renderer in
      let p = (p :> Papget.item) in
      register_papget p p
	
  | _ -> failwith (sprintf "Unexpected papget type: %s" type_)
	

exception Parse_message_dnd of string
(* Drag and drop handler for papgets *)
let parse_message_dnd =
  let sep = Str.regexp ":" in
  fun s ->
    match Str.split sep s with
      [s; c; m; f] -> (s, c, m, f)
    | _ -> raise (Parse_message_dnd (Printf.sprintf "parse_dnd: %s" s))
let dnd_data_received = fun canvas_group context ~x ~y data ~info ~time ->
  try (* With the format sent by Messages *)
    let (sender, class_name, msg_name, field_name) = parse_message_dnd data#data in
    let sender = if sender = "*" then None else Some sender in
    let msg_listener = new Papget.message ~class_name ?sender msg_name in
    let renderer = new Papget.canvas_text canvas_group (float x) (float y) in
    let p = new Papget.canvas_display_item msg_listener field_name (renderer:> Papget.renderer) in
    let p = (p :> Papget.item) in
    register_papget p p
  with
    Parse_message_dnd _ ->
      try (* XML spec *)
	let xml = Xml.parse_string data#data in
	(* Add x and y attributes *)
	let attrs = Xml.attribs xml @ ["x", string_of_int x; "y", string_of_int y] in
	let papget_xml = Xml.Element (Xml.tag xml,attrs,Xml.children xml) in
	create canvas_group papget_xml
      with
	exc -> prerr_endline (Printexc.to_string exc)
