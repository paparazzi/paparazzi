(*
 * Flight plan preprocessing (procedure including)
 *
 * Copyright (C) 2004-2009 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

module G2D = Geometry_2d

open Expr_syntax

let rec list_split3 = function
[] -> ([], [], [])
  | (x,y,z)::l ->
    let (rx, ry, rz) = list_split3 l in (x::rx, y::ry, z::rz)

let parse_expression = fun s ->
  let lexbuf = Lexing.from_string s in
  try
    Expr_parser.expression Expr_lexer.token lexbuf
  with
      Failure _ ->
        fprintf stderr "Lexing error in '%s': unexpected char: '%c' \n"
          s (Lexing.lexeme_char lexbuf 0);
        exit 1
    | Parsing.Parse_error ->
      fprintf stderr "Fp_proc: Parsing error in '%s', token '%s' ?\n"
        s (Lexing.lexeme lexbuf);
      exit 1


open Latlong

let subst_expression = fun (name, env) e ->
  let rec sub = fun e ->
    match e with
        Ident i -> Ident (try List.assoc i env with Not_found -> i)
      | Int _ | Float _ | Field _ -> e
      | Call ("IndexOfBlock", [Ident s]) -> Call ("IndexOfBlock", [ Ident ("'"^name^"."^s^"'") ]) (* prefix block name *)
      | Call (i, es) -> Call (i, List.map sub es)
      | CallOperator (i, es) -> CallOperator (i, List.map sub es)
      | Index (i,e) -> Index (i,sub e)
      | Deref (e,f) -> Deref (sub e, f) in
  sub e


let transform_expression = fun env e ->
  let e' = subst_expression env e in
  Expr_syntax.sprint e'


let transform_values = fun attribs_not_modified env attribs ->
  List.map
    (fun (a, v) ->
      let v' =
        if List.mem (String.lowercase_ascii a) attribs_not_modified
        then v
        else transform_expression env (parse_expression v) in
      (a, v'))
    attribs


let prefix_or_deroute = fun prefix reroutes name attribs ->
  List.map
    (fun (a, v) ->
      let v' =
        if String.lowercase_ascii a = name then
          try List.assoc v reroutes with
              Not_found -> prefix v
        else v in
      (a, v'))
    attribs

let transform_exception = fun prefix reroutes env xml  ->
  match xml with
      Xml.Element (tag, attribs, children) ->
        assert (children=[]);
        let attribs = prefix_or_deroute prefix reroutes "deroute" attribs in
        let attribs = transform_values [] env attribs in
        Xml.Element (tag, attribs, children)
    | _ -> failwith "transform_exception"



let transform_stage = fun prefix reroutes env xml ->
  let rec tr = fun xml ->
    match xml with
        Xml.Element (tag, attribs, children) -> begin
          match String.lowercase_ascii tag with
              "exception" ->
                transform_exception prefix reroutes env xml
            | "while" ->
              let attribs = transform_values [] env attribs in
              Xml.Element (tag, attribs, List.map tr children)
            | "for" ->
              let attribs = transform_values [] env attribs in
              Xml.Element (tag, attribs, List.map tr children)
            | "heading" ->
              assert (children=[]);
              let attribs = transform_values ["vmode"] env attribs in
              Xml.Element (tag, attribs, children)
            | "attitude" ->
              let attribs = transform_values ["vmode"] env attribs in
              Xml.Element (tag, attribs, children)
            | "manual" ->
              let attribs = transform_values [] env attribs in
              Xml.Element (tag, attribs, children)
            | "return" ->
              let attribs = transform_values ["reset_stage"] env attribs in
              Xml.Element (tag, attribs, children)
            | "go" ->
              assert (children=[]);
              let attribs = transform_values ["wp";"from";"hmode";"vmode"] env attribs in
              Xml.Element (tag, attribs, children)
            | "xyz" ->
              assert (children=[]);
              let attribs = transform_values [] env attribs in
              Xml.Element (tag, attribs, children)
            | "circle" ->
              assert (children=[]);
              let attribs = transform_values ["wp";"hmode";"vmode"] env attribs in
              Xml.Element (tag, attribs, children)
            | "eight" ->
              let attribs = transform_values ["center";"turn_around";"radius"] env attribs in
              Xml.Element (tag, attribs, children)
            | "deroute" ->
              assert (children=[]);
              let attribs = prefix_or_deroute prefix reroutes "block" attribs in
              Xml.Element (tag, attribs, children)
            | "stay" ->
              assert (children=[]);
              let attribs = transform_values ["wp"; "vmode"] env attribs in
              Xml.Element (tag, attribs, children)
            | "call" | "call_once" | "set" ->
              let attribs = transform_values ["var"] env attribs in
              Xml.Element (tag, attribs, children)
            | _ -> failwith (sprintf "Fp_proc: Unexpected tag: '%s'" tag)
        end
      | _ -> failwith "Fp_proc: Xml.Element expected"
  in
  tr xml

let transform_block = fun prefix reroutes env xml ->
  let stages = List.map (transform_stage prefix reroutes env) (Xml.children xml) in
  let block = Xml.Element("block", Xml.attribs xml, stages) in
  ExtXml.subst_attrib "name" (prefix (ExtXml.attrib xml "name")) block


let build_assocs = fun tag key_attr val_attr xml ->
  let xmls =
    List.filter
      (fun x -> ExtXml.tag_is x tag)
      (Xml.children xml) in

  List.map
    (fun xml -> (ExtXml.attrib xml key_attr, ExtXml.attrib xml val_attr))
    xmls


let get_children = fun tag xml ->
  try Xml.children (ExtXml.child xml tag) with Not_found -> []


let get_pc_data = fun tag xml ->
  try
    Xml.pcdata (ExtXml.child (ExtXml.child xml tag) "0")
  with
      Not_found -> ""


let append_children = fun (tag, new_children) xml ->
  let children = get_children tag xml @ new_children in
  let new_elt = Xml.Element (tag, [], children) in
  ExtXml.subst_or_add_child tag new_elt xml

let append_pc_data = fun tag new_data xml ->
  let data = get_pc_data tag xml ^ "\n" ^ new_data in
  let new_elt = Xml.Element (tag, [], [Xml.PCData data]) in
  ExtXml.subst_or_add_child tag new_elt xml



let parse_include = fun dir flight_plan include_xml ->
  let f =
    let procedure = ExtXml.attrib include_xml "procedure" in
    try Ocaml_tools.find_file [dir; Env.flight_plans_path] procedure with
        Not_found ->
          failwith (sprintf "parse_include: %s not found\n" procedure) in
  let proc_name = ExtXml.attrib include_xml "name" in
  let prefix = fun x -> proc_name ^ "." ^ x in

  let reroutes = build_assocs "with" "from" "to" include_xml
  and args_assocs = build_assocs "arg" "name" "value" include_xml in

  try
    let proc = ExtXml.parse_file ~noprovedtd:true f in
    let params = List.filter
      (fun x -> ExtXml.tag_is x "param")
      (Xml.children proc) in

    (* Build the environment with arguments and default values *)
    let make_assoc = fun xml ->
      let name = ExtXml.attrib xml "name" in
      try
        (name, List.assoc name args_assocs)
      with
          Not_found ->
            try
              (name, Xml.attrib xml "default_value")
            with
                _  -> failwith (sprintf "Value required for param '%s' in %s" name (Xml.to_string include_xml)) in
    let env =  List.map make_assoc params in

    let waypoints = get_children "waypoints" proc
    and exceptions = get_children "exceptions" proc
    and modules = get_children "modules" proc
    and blocks = get_children "blocks" proc
    and sectors = get_children "sectors" proc
    and header = get_pc_data "header" proc in

    let exceptions = List.map (transform_exception prefix reroutes (proc_name, env)) exceptions
    and blocks = List.map (transform_block prefix reroutes (proc_name, env)) blocks in

    List.fold_right
      append_children
      ["waypoints", waypoints;
       "blocks", blocks;
       "modules", modules;
       "exceptions", exceptions;
       "sectors", sectors]
      (append_pc_data "header" header flight_plan)
  with
      Failure msg -> fprintf stderr "Error: %s\n" msg; exit 1



let replace_children = fun xml new_children_assoc ->
  Xml.Element (Xml.tag xml, Xml.attribs xml,
               List.map
                 (fun x ->
                   try
                     let new_children = List.assoc (Xml.tag x) new_children_assoc in
                     new_children
                   with
                       Not_found -> x
                 )
                 (Xml.children xml))


let process_includes = fun dir xml ->
  let includes =
    try Xml.children (ExtXml.child xml "includes") with Not_found -> []
  and xml_without_includes = ExtXml.remove_child "includes" xml in

  List.fold_left (parse_include dir) xml_without_includes includes



let remove_attribs = fun xml names ->
  List.filter (fun (x,_) -> not (List.mem (String.lowercase_ascii x) names)) (Xml.attribs xml)

let xml_assoc_attrib = fun a v xmls ->
  List.find (fun x -> ExtXml.attrib x a = v) xmls

let g2D_of_waypoint = fun wp ->
  { G2D.x2D = ExtXml.float_attrib wp "x"; y2D =  ExtXml.float_attrib wp "y" }

let new_waypoint = fun wp qdr dist waypoints ->
  let wp_xml = xml_assoc_attrib "name" wp !waypoints in
  let wp2D = g2D_of_waypoint wp_xml in
  let a = (Deg>>Rad)(90. -. qdr) in
  let xy = G2D.vect_add wp2D (G2D.polar2cart { G2D.r2D = dist; theta2D = a }) in
  let x = string_of_float xy.G2D.x2D
  and y = string_of_float xy.G2D.y2D in
  let name = sprintf "%s_%.0f_%.0f" wp qdr dist in
  let alt = try ["alt", Xml.attrib wp_xml "alt"] with _ -> [] in
  waypoints := Xml.Element("waypoint", ["name", name; "x", x; "y", y]@alt, []) :: !waypoints;
  name


let replace_wp = fun stage waypoints ->
  try
    let qdr = ExtXml.float_attrib stage "wp_qdr"
    and dist = ExtXml.float_attrib stage "wp_dist" in
    let wp = ExtXml.attrib stage "wp" in

    let name = new_waypoint wp qdr dist waypoints in

    let other_attribs = remove_attribs stage ["wp";"wp_qdr";"wp_dist"] in
    Xml.Element (Xml.tag stage, ("wp", name)::other_attribs, [])
  with
      _ -> stage


let replace_from = fun stage waypoints ->
  try
    let qdr = ExtXml.float_attrib stage "from_qdr"
    and dist = ExtXml.float_attrib stage "from_dist" in
    let wp = ExtXml.attrib stage "from" in

    let name = new_waypoint wp qdr dist waypoints in

    let other_attribs = remove_attribs stage ["from";"from_qdr";"from_dist"] in
    Xml.Element (Xml.tag stage, ("from", name)::other_attribs, [])
  with
      _ -> stage


let process_stage = fun stage waypoints ->
  let rec do_it = fun stage ->
    match String.lowercase_ascii (Xml.tag stage) with
        "go" | "stay" | "circle" ->
          replace_from (replace_wp stage waypoints) waypoints

      | "while" ->
        Xml.Element("while", Xml.attribs stage, List.map do_it (Xml.children stage))
      | _ -> stage in
  do_it stage


let process_relative_waypoints = fun xml ->
  let waypoints = ExtXml.child xml "waypoints"
  and blocks = ExtXml.child xml "blocks" in

  let blocks_list = Xml.children blocks in

  let waypoints_list = ref (Xml.children waypoints) in

  let blocks_list =
    List.map
      (fun block ->
        let new_children =
          List.map
            (fun stage -> process_stage stage waypoints_list)
            (Xml.children block) in
        Xml.Element (Xml.tag block, Xml.attribs block, new_children)
      )
      blocks_list in

  let new_waypoints = Xml.Element ("waypoints", Xml.attribs waypoints, !waypoints_list)
  and blocks = Xml.Element ("blocks", Xml.attribs blocks, blocks_list) in

  replace_children xml ["waypoints", new_waypoints; "blocks", blocks]


let regexp_path = Str.regexp "[ \t,]+"


let rec stage_process_path = fun stage rest ->
  if Xml.tag stage = "path" then
    let waypoints = Str.split regexp_path (ExtXml.attrib stage "wpts") in
    let attribs = Xml.attribs stage in
    let rec loop = function
      | [] -> failwith "Waypoint expected in path stage"
      | [wp] -> (* Just go to this single point *)
        Xml.Element("go", ("wp", wp)::attribs, [])::rest
      | wp1::wp2::ps ->
        Xml.Element("go", ["from", wp1;
                           "hmode","route";
                           "wp", wp2]@attribs, [])::
          if ps = [] then rest else loop (wp2::ps) in
    loop waypoints
  else if Xml.tag stage = "for" || Xml.tag stage = "while" then
    let attribs = Xml.attribs stage in
    Xml.Element(Xml.tag stage, attribs, List.fold_right stage_process_path (Xml.children stage) [])::rest
  else
    stage::rest

let block_process_path = fun block ->
  let stages = Xml.children block in
  let new_stages = List.fold_right stage_process_path stages [] in
  Xml.Element (Xml.tag block, Xml.attribs block, new_stages)


let process_paths = fun xml ->
  let blocks = ExtXml.child xml "blocks" in
  let blocks_list = List.map block_process_path (Xml.children blocks) in
  let new_blocks = Xml.Element ("blocks", Xml.attribs blocks, blocks_list) in
  replace_children xml ["blocks", new_blocks]
