(*
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
module PC = Papget_common

let filter_acid = fun save conf ->
  let filtered = List.filter (fun x ->
    (* keep element if save is true or save is false and attrib name is not ac_id *)
    if (ExtXml.attrib_or_default x "name" "" = "ac_id") && (not save) then false
    else true) (Xml.children conf) in
  Xml.Element (Xml.tag conf, Xml.attribs conf, filtered)

let papgets = Hashtbl.create 5
let register_papget = fun p -> Hashtbl.add papgets p p
let dump_store = fun save_id ->
  Hashtbl.fold
    (fun _ p r ->
      if not p#deleted then
        (filter_acid save_id (p#config ()))::r
      else
        r)
    papgets
    []

let has_papgets = fun () ->
  (Hashtbl.fold (fun _ p n -> if p#deleted then n else n + 1) papgets 0) > 0

let papget_listener =
  let sep = Str.regexp "[:\\.]" in
  fun papget ->
    try
      let field = Papget_common.get_property "field" papget in
      let sender = try Some (Papget_common.get_property "ac_id" papget) with _ -> None in
      match Str.split sep field with
          [msg_name; field_name] ->
            (new Papget.message_field ?sender msg_name field_name)
        | _ -> failwith (sprintf "Unexpected field spec: %s" field)
    with
        _ -> failwith (sprintf "field attr expected in '%s" (Xml.to_string papget))


let block_name_of_index = function
[ i ] ->
  let i = sprintf "%.0f" (float_of_string i) in
  if Hashtbl.length Live.aircrafts = 1 then
    Hashtbl.fold
      (fun ac_id ac _r ->
        let blocks = ExtXml.child ac.Live.fp "blocks" in
        let block = ExtXml.child blocks i  in
        ExtXml.attrib block "name")
      Live.aircrafts
      "N/A"
  else
    "N/A"
  | _ -> failwith "Papgets.block_name_of_index"

let extra_functions =
  ["BlockName", block_name_of_index ]


let expression_listener = fun papget ->
  let expr = Papget_common.get_property "expr" papget in
  let expr = Expr_lexer.parse expr in
  let sender = try Some (Papget_common.get_property "ac_id" papget) with _ -> None in
  new Papget.expression ~extra_functions ?sender expr



let display_float_papget = fun canvas_group config display x y listener ->
  let renderer =
    match display with
        "text" ->
          (new Papget_renderer.canvas_text ~config canvas_group x y :> Papget_renderer.t)
      | "ruler" ->
        (new Papget_renderer.canvas_ruler canvas_group ~config x y :> Papget_renderer.t)
      | "gauge" ->
        (new Papget_renderer.canvas_gauge ~config canvas_group x y :> Papget_renderer.t)
      | "led" ->
        (new Papget_renderer.canvas_led ~config canvas_group x y :> Papget_renderer.t)
      | _ -> failwith (sprintf "Unexpected papget display: %s" display) in

  let p = new Papget.canvas_display_float_item ~config listener renderer in
  let p = (p :> Papget.item) in
  register_papget p



let locked = fun config ->
  try
    [PC.property "locked" (PC.get_property "locked" config)]
  with _ -> []

let ac_id_prop = fun config ->
  try
    [PC.property "ac_id" (PC.get_property "ac_id" config)]
  with _ -> []

let create = fun canvas_group zoom_adj papget ->
  try
    let type_ = ExtXml.attrib papget "type"
    and display = ExtXml.attrib papget "display"
    and x = ExtXml.float_attrib papget "x"
    and y = ExtXml.float_attrib papget "y"
    and config = Xml.children papget in
    match type_ with
        "expression" ->
          let expr_listener = expression_listener papget in
          display_float_papget canvas_group config display x y expr_listener

      | "message_field" ->
        let msg_listener = papget_listener papget in
        display_float_papget canvas_group config display x y msg_listener

      | "goto_block" ->
        let renderer =
          match display with
              "button" ->
                (new Papget_renderer.canvas_button canvas_group ~config x y :> Papget_renderer.t)
            | _ -> failwith (sprintf "Unexpected papget display: %s" display) in
        let block_name = Papget_common.get_property "block_name" papget in
        let clicked = fun () ->
          let jump_to_block = fun ac_id ac ->
            let blocks = ExtXml.child ac.Live.fp "blocks" in
            let block = ExtXml.child ~select:(fun x -> ExtXml.attrib x "name" = block_name) blocks "block" in
            let block_id = ExtXml.int_attrib block "no" in
            Live.jump_to_block ac_id block_id
          in
          let sender = try Some (Papget_common.get_property "ac_id" papget) with _ -> None in
          match sender with
            Some ac_id -> begin try jump_to_block ac_id (Hashtbl.find Live.aircrafts ac_id) with _ -> () end
          | None ->
              prerr_endline "Warning: goto_block papget sends to all active A/C";
              Hashtbl.iter jump_to_block Live.aircrafts
        in
        let properties =
          [ Papget_common.property "block_name" block_name ] @ locked papget @ ac_id_prop papget in

        let p = new Papget.canvas_goto_block_item properties clicked renderer in
        let p = (p :> Papget.item) in
        register_papget p
      | "variable_setting" ->
        let renderer =
          match display with
              "button" ->
                (new Papget_renderer.canvas_button canvas_group ~config x y :> Papget_renderer.t)
            | _ -> failwith (sprintf "Unexpected papget display: %s" display) in

        let varname = Papget_common.get_property "variable" papget
        and value = float_of_string (Papget_common.get_property "value" papget) in

        let clicked = fun () ->
          let send_setting = fun ac_id ac ->
            match ac.Live.dl_settings_page with
              None -> ()
            | Some settings ->
                let var_id = settings#assoc varname in
                Live.dl_setting ac_id var_id value
          in
          let sender = try Some (Papget_common.get_property "ac_id" papget) with _ -> None in
          match sender with
            Some ac_id -> begin try send_setting ac_id (Hashtbl.find Live.aircrafts ac_id) with _ -> () end
          | None ->
              prerr_endline "Warning: variable_setting papget sending to all active A/C";
              Hashtbl.iter send_setting Live.aircrafts
        in
        let properties =
          [ Papget_common.property "variable" varname;
            Papget_common.float_property "value" value ]
          @ locked papget @ ac_id_prop papget in
        let p = new Papget.canvas_variable_setting_item properties clicked renderer in
        let p = (p :> Papget.item) in
        register_papget p

      | "video_plugin" ->
        let renderer =
          match display with
              "mplayer" ->
                (new Papget_renderer.canvas_mplayer canvas_group ~config x y :> Papget_renderer.t)
            | "plugin" ->
              (new Papget_renderer.canvas_plugin canvas_group ~config x y :> Papget_renderer.t)
            | _ -> failwith (sprintf "Unexpected papget display: %s" display) in

        let properties = locked papget in
        let p = new Papget.canvas_video_plugin_item properties renderer zoom_adj in
        let p = (p :> Papget.item) in
        register_papget p

      | _ -> failwith (sprintf "Unexpected papget type: %s" type_)
  with
      exc -> fprintf stderr "Papgets.create: %s\n%!" (Printexc.to_string exc)


exception Parse_message_dnd of string
(* Drag and drop handler for papgets *)
let parse_message_dnd =
  let sep = Str.regexp ":" in
  fun s ->
    match Str.split sep s with
        [s; c; m; f;scale] -> (s, c, m, f,scale)
      | _ -> raise (Parse_message_dnd (Printf.sprintf "parse_dnd: %s" s))
let dnd_data_received = fun canvas_group zoom_adj _context ~x ~y data ~info ~time ->
  try (* With the format sent by Messages *)
    let (sender, _class_name, msg_name, field_name,scale) = parse_message_dnd data#data in
    let attrs =
      [ "type", "message_field";
        "display", "text";
        "x", sprintf "%d" x; "y", sprintf "%d" y ]
    and props =
      [ Papget_common.property "field" (sprintf "%s:%s" msg_name field_name);
        Papget_common.property "ac_id" sender;
        Papget_common.property "scale" scale ] in
    let papget_xml = Xml.Element ("papget", attrs, props) in
    create canvas_group zoom_adj papget_xml
  with
      Parse_message_dnd _ ->
        try (* XML spec *)
          let xml = Xml.parse_string data#data in
    (* Add x and y attributes *)
          let attrs = Xml.attribs xml @ ["x", string_of_int x; "y", string_of_int y] in
          let papget_xml = Xml.Element (Xml.tag xml,attrs,Xml.children xml) in
          create canvas_group zoom_adj papget_xml
        with
            exc -> prerr_endline (Printexc.to_string exc)
