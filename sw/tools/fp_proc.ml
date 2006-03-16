(*
 * $Id$
 *
 * Flight plan preprocessing (procedure including)
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

module G2D = Geometry_2d

open Fp_syntax

let nop_stage = Xml.Element ("while", ["cond","FALSE"],[])


let parse_expression = fun s ->
  let lexbuf = Lexing.from_string s in
  try
    Fp_parser.expression Fp_lexer.token lexbuf
  with
    Failure("lexing: empty token") ->
      Printf.fprintf stderr "Lexing error in '%s': unexpected char: '%c' \n"
	s (Lexing.lexeme_char lexbuf 0);
      exit 1
  | Parsing.Parse_error ->
      Printf.fprintf stderr "Parsing error in '%s', token '%s' ?\n"
	s (Lexing.lexeme lexbuf);
      exit 1


open Latlong
let norm_2pi = fun f -> if f < 0. then f +. 2. *. pi else f

(* Translation and rotation *)
type affine = { dx : float; dy : float; angle : float (* Deg Clockwise *) }

let dtd_error = fun f e ->
  Printf.fprintf stderr "DTD error in '%s': %s\n" f e;
  exit 1

(* Rotation. Would be better with a matrix multiplication ? *)
let rotate = fun angle (x, y) ->
  let angle = -. (Deg>>Rad) angle in
  let a = atan2 y x
  and r = sqrt (x**2. +. y**2.) in
  let a' = a +. angle in
  (r*.cos a', r*.sin a')

let rotate_expression = fun a expression ->
  let rec rot = fun e ->
    match e with
    | Call("Qdr", [Float a']) -> Call("Qdr", [Float (a' +. a)])
    | Call("Qdr", [Int a']) -> Call("Qdr", [Float (float a' +. a)])
    | Call(op, [e1; e2]) when op = "And" || op ="Or" ->
	Call(op, [rot e1; rot e2]) 
    | CallOperator(op, [e]) -> CallOperator(op, [rot e])
    | CallOperator(op, [e1; e2]) -> CallOperator(op, [rot e1; rot e2])
    | CallOperator(_op, _) -> failwith "fp_proc: Operator should be unary or binary"
    | _ -> e in
  rot expression

let subst_expression = fun env e ->
  let rec sub = fun e ->
    match e with
      Ident i -> Ident (try List.assoc i env with Not_found -> i)
    | Int _ | Float _ -> e
    | Call (i, es) -> Call (i, List.map sub es)
    | CallOperator (i, es) -> CallOperator (i, List.map sub es)
    | Index (i,e) -> Index (i,sub e) in
  sub e


let transform_expression = fun affine env e ->
  let e' = rotate_expression affine.angle e in
  let e'' = subst_expression env e' in
  Fp_syntax.sprint_expression e''
  

let transform_values = fun attribs_not_modified affine env attribs ->
  List.map
    (fun (a, v) ->
      let e = parse_expression v in
      let e' = 
	if String.lowercase a = "course" 
	then CallOperator("+", [e; Float affine.angle])
	else e in
      let v' =
	if List.mem (String.lowercase a) attribs_not_modified
	then v
	else transform_expression affine env e' in
      (a, v'))
    attribs

let transform_waypoint = fun prefix affine xml ->
  let x = ExtXml.float_attrib xml "x"
  and y = ExtXml.float_attrib xml "y" in
  let (x, y) = rotate affine.angle (x, y) in
  let (x, y) = (x+.affine.dx, y+.affine.dy) in
  let alt = try ["alt", ExtXml.attrib xml "alt"] with ExtXml.Error _ -> [] in
  Xml.Element (Xml.tag xml,
	       ["name", prefix (ExtXml.attrib xml "name");
		"x", string_of_float x;
		"y", string_of_float y]@alt,
	       [])


let prefix_value = fun prefix name attribs ->
  List.map
    (fun (a, v) ->
      let v' = if String.lowercase a = name then prefix v else v in
      (a, v'))
    attribs

let prefix_or_deroute = fun prefix reroutes name attribs ->
  List.map
    (fun (a, v) ->
      let v' =
	if String.lowercase a = name then
	  try List.assoc v reroutes with
	    Not_found -> prefix v
	else v in
      (a, v'))
    attribs

let transform_attribs = fun affine attribs ->
  List.map
    (fun (a, v) ->
      match String.lowercase a with
	"wp_qdr" | "from_qdr" ->
	  (a, string_of_float (float_of_string v +. affine.angle))
      | _ -> (a, v)
    )
    attribs


let transform_stage = fun prefix reroutes affine env xml ->
  let rec tr = fun xml ->
    match xml with
      Xml.Element (tag, attribs, children) -> begin
	match tag with
	  "exception" ->
	    assert (children=[]);
	    let attribs = prefix_or_deroute prefix reroutes "deroute" attribs in
	    let attribs = transform_values [] affine env attribs in
	    Xml.Element (tag, attribs, children)
	| "while" ->
	    let attribs = transform_values [] affine env attribs in
	  Xml.Element (tag, attribs, List.map tr children)
	| "heading" ->
	    assert (children=[]);
	    let attribs = transform_values ["vmode"] affine env attribs in
	    Xml.Element (tag, attribs, children)
	| "go" ->
	  assert (children=[]);
	  let attribs = transform_values ["wp";"from";"hmode";"vmode"] affine env attribs in
	  let attribs = prefix_value prefix "wp" attribs in
	  let attribs = prefix_value prefix "from" attribs in
	  let attribs = transform_attribs affine attribs in
	  Xml.Element (tag, attribs, children)
      | "xyz" ->
	  assert (children=[]);
	  let attribs = transform_values [] affine env attribs in
	  Xml.Element (tag, attribs, children)
      | "circle" ->
	  assert (children=[]);
	  let attribs = transform_values ["wp";"hmode";"vmode"] affine env attribs in
	  let attribs = prefix_value prefix "wp" attribs in
	  let attribs = transform_attribs affine attribs in
	  Xml.Element (tag, attribs, children)
      | "deroute" ->
	  assert (children=[]);
	  let attribs = prefix_or_deroute prefix reroutes "block" attribs in
	  Xml.Element (tag, attribs, children)
      | "stay" ->
	  assert (children=[]);
	  let attribs = transform_values ["wp"; "vmode"] affine env attribs in
	  let attribs = prefix_value prefix "wp" attribs in
	  let attribs = transform_attribs affine attribs in
	  Xml.Element (tag, attribs, children)
      | _ -> failwith (Printf.sprintf "Fp_proc: Unexpected tag: '%s'" tag)
    end
  | _ -> failwith "Fp_proc: Xml.Element expected"
  in
  tr xml

let transform_block = fun prefix reroutes affine env xml ->
  Xml.Element (Xml.tag xml,
	       ["name", prefix (ExtXml.attrib xml "name")],
	       List.map (transform_stage prefix reroutes affine env) (Xml.children xml))
  

let parse_include = fun dir include_xml ->
  let f = Filename.concat dir (ExtXml.attrib include_xml "procedure") in
  let proc_name = ExtXml.attrib include_xml "name" in
  let prefix = fun x -> proc_name ^ "." ^ x in
  let affine = { 
    dx = ExtXml.float_attrib  include_xml "x";
    dy = ExtXml.float_attrib  include_xml "y";
    angle = ExtXml.float_attrib  include_xml "rotate"
  } in
  let reroutes = 
    List.filter 
      (fun x -> String.lowercase (Xml.tag x) = "with") 
      (Xml.children include_xml) in
  let reroutes = List.map
      (fun xml -> (ExtXml.attrib xml "from", ExtXml.attrib xml "to"))
      reroutes in
  let args = 
    List.filter 
      (fun x -> String.lowercase (Xml.tag x) = "arg") 
      (Xml.children include_xml) in
  let env = List.map
      (fun xml -> (ExtXml.attrib xml "name", ExtXml.attrib xml "value"))
      args in
  try
    let proc = Xml.parse_file f in
    let params = List.filter 
	(fun x -> String.lowercase (Xml.tag x) = "param")
	(Xml.children proc) in
    let value = fun xml env ->
      let name = ExtXml.attrib xml "name" in
      try
	(name, List.assoc name env)
      with
	Not_found ->
	  try
	    (name, Xml.attrib xml "default_value")
	  with
	    _  -> failwith (Printf.sprintf "Value required for param '%s' in %s" name (Xml.to_string include_xml)) in
    (* Complete the environment with default values *)
    let env =  List.map (fun xml -> value xml env) params in

    let waypoints = Xml.children (ExtXml.child proc "waypoints")
    and blocks = Xml.children (ExtXml.child proc "blocks") in

    let waypoints = List.map (transform_waypoint prefix affine) waypoints in
    let blocks = List.map (transform_block prefix reroutes affine env) blocks in
    (waypoints, blocks)
  with
    Dtd.Prove_error e -> dtd_error f (Dtd.prove_error e)
  | Dtd.Check_error e -> dtd_error f (Dtd.check_error e)
      

(** Adds new children to a list of XML elements *)
let insert_children = fun xmls new_children_assoc ->
  List.map
    (fun x ->
      try
	let new_children = List.assoc (Xml.tag x) new_children_assoc
	and old_children = Xml.children x in
	Xml.Element (Xml.tag x, Xml.attribs x, old_children @ new_children)
      with
	Not_found -> x
    )
    xmls
    
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
  let includes, children =
   List.partition (fun x -> Xml.tag x = "include") (Xml.children xml) in

  (* List of pairs of list (waypoints, blocks) *)
  let waypoints_and_blocks = List.map (parse_include dir) includes in

  let (inc_waypoints, inc_blocks) = List.split waypoints_and_blocks in
  let inc_waypoints = List.flatten inc_waypoints
  and inc_blocks = List.flatten inc_blocks in

  let new_children = insert_children children
      ["waypoints", inc_waypoints; "blocks", inc_blocks] in

  Xml.Element (Xml.tag xml, Xml.attribs xml, new_children)

let remove_attribs = fun xml names ->
  List.filter (fun (x,_) -> not (List.mem (String.lowercase x) names)) (Xml.attribs xml)

let xml_assoc_attrib = fun a v xmls ->
  List.find (fun x -> ExtXml.attrib x a = v) xmls

let coords_of_waypoint = fun wp ->
  (ExtXml.float_attrib wp "x", ExtXml.float_attrib wp "y")

let coords_of_wp_name = fun wp waypoints ->
  let wp = xml_assoc_attrib "name" wp waypoints in
  (ExtXml.float_attrib wp "x", ExtXml.float_attrib wp "y")

let new_waypoint = fun wp qdr dist waypoints ->
  let wp_xml = xml_assoc_attrib "name" wp !waypoints in
  let wpx, wpy = coords_of_waypoint wp_xml in
  let a = (Deg>>Rad)(90. -. qdr) in
  let x = string_of_float (wpx +. dist *. cos a)
  and y = string_of_float (wpy +. dist *. sin a) in
  let name = Printf.sprintf "%s_%.0f_%.0f" wp qdr dist in
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
    match String.lowercase (Xml.tag stage) with
      "go" | "stay" | "circle" ->
	replace_from (replace_wp stage waypoints) waypoints
	  
    | "while" ->
	Xml.Element("while", Xml.attribs stage, List.map do_it (Xml.children stage))
    | _ -> stage in
  do_it stage
  
  
let process_relative_waypoints = fun xml ->
  let waypoints = (ExtXml.child xml "waypoints")
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



(** Path preprocessing: a list of waypoints is translated into an alternance of
  route and circle stages *)
let compile_path = fun wpts radius last_last last ps rest ->
  let rec loop = fun p0 last ps ->
    match ps with
      [] -> rest
    | p::ps ->
	let wp = Xml.attrib p "wp" in
	let (x1, y1) = coords_of_wp_name last wpts
	and (x2, y2) = coords_of_wp_name wp wpts in
	let p1 = {G2D.x2D=x1; y2D=y1}
	and p2 = {G2D.x2D=x2; y2D=y2} in
	let (c, f, s) = G2D.arc_segment p0 p1 p2 radius in
	  
	(* Angle between P1 and F *)
	let alpha_cf = (G2D.cart2polar (G2D.vect_make c f)).G2D.theta2D in
	let alpha_c1 = (G2D.cart2polar (G2D.vect_make c p1)).G2D.theta2D in
	let alpha_fc1 = norm_2pi (-. s *. (alpha_c1 -. alpha_cf)) in

	let theta = abs_float (alpha_fc1) /. 2. /. pi in

	(* C relative to P1, F relative to P2 *)
	let alpha_1c = (G2D.cart2polar (G2D.vect_make p1 c)).G2D.theta2D
	and alpha_2f = (G2D.cart2polar (G2D.vect_make p2 f)).G2D.theta2D
	and d_f2 = G2D.distance f p2 in
	let c_last_qdr= norm_2pi (pi /. 2. -. alpha_1c)
	and f_wp_qdr= norm_2pi (pi /. 2. -. alpha_2f) in
	let until = Printf.sprintf "(circle_count > %f)" theta in
	let sradius = string_of_float (-. s *. radius) in

	Xml.Element ("circle", ["wp", last;
				"wp_qdr", string_of_float ((Rad>>Deg)c_last_qdr);
				"wp_dist", string_of_float radius;
		                "radius", sradius;
				"until", until],[])::
	Xml.Element ("go", ["from",wp; 
			    "from_qdr", string_of_float ((Rad>>Deg)f_wp_qdr); 
			    "from_dist", string_of_float d_f2;
			    "hmode", "route";
			    "approaching_time", "2";
			    "wp", wp], [])::
	loop f wp ps in
  loop last_last last ps;;
  

let stage_process_path = fun wpts stage rest ->
  if Xml.tag stage = "path" then
    let radius = float_of_string (ExtXml.attrib stage "radius") in
    match Xml.children stage with
      [] -> nop_stage::rest
    | [p] -> (* Just go to this single point *)
	Xml.Element("go", ["wp", Xml.attrib p "wp"], [])::rest
    | p1::p2::ps -> 
        (* Start from a route from the first to the second point *)
	let wp1 = Xml.attrib p1 "wp"
	and wp2 = Xml.attrib p2 "wp" in
	Xml.Element("go", ["from", wp1;
			   "hmode","route";
			   "wp", wp2], [])::
	(* Here starts the actual translation *)
	let x1, y1 = coords_of_wp_name wp1 wpts in
	let p1 = {Geometry_2d.x2D=x1; y2D=y1} in
	compile_path wpts radius p1 wp2 ps rest
  else
    stage::rest

let block_process_path = fun wpts block ->
  let stages = Xml.children block in
  let new_stages = List.fold_right (stage_process_path wpts) stages [] in
  Xml.Element (Xml.tag block, Xml.attribs block, new_stages)
  

let process_paths = fun xml ->
  let waypoints = Xml.children (ExtXml.child xml "waypoints")
  and blocks = ExtXml.child xml "blocks" in
  let blocks_list = List.map (block_process_path waypoints) (Xml.children blocks) in
  let new_blocks = Xml.Element ("blocks", Xml.attribs blocks, blocks_list) in
  replace_children xml ["blocks", new_blocks]
