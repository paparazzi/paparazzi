(*
 * $Id$
 *
 * Flight plan preprocessing (from XML to C)
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

open Printf
open Latlong

let check_expressions = ref true

let parse_expression = Fp_proc.parse_expression

let parse = fun s ->
  if !check_expressions then
    let e = parse_expression s in
    let unexpected = fun kind x ->
      fprintf stderr "Paring error in '%s': unexpected %s: '%s' \n" s kind x;
      exit 1 in
    begin
      try
	Fp_syntax.check_expression e
      with
	Fp_syntax.Unknown_operator x -> unexpected "operator" x
      | Fp_syntax.Unknown_ident x -> unexpected "ident" x
      | Fp_syntax.Unknown_function x -> unexpected "function" x
    end;
    Fp_syntax.sprint_expression e
  else
    s

let parsed_attrib = fun xml a ->
  parse (ExtXml.attrib xml a)

let pi = atan 1. *. 4.

let radE8_of_deg = fun d ->
  d /. 180. *. pi *. 1e8

let rad_of_deg = fun d ->
  d /. 180. *. pi

let gen_label =
  let x = ref 0 in
  fun p -> incr x; sprintf "%s_%d" p !x

let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun f ->
  printf "%s" (String.make !margin ' ');
  printf f

let float_attrib = fun xml a -> float_of_string (Xml.attrib xml a)
let int_attrib = fun xml a -> int_of_string (Xml.attrib xml a)
let name_of = fun wp -> ExtXml.attrib wp "name"


let ground_alt = ref 0.
let security_height = ref 0.

let check_altitude = fun a x ->
  if a < !ground_alt +. !security_height then begin
    fprintf stderr "\nWARNING: low altitude (%.0f<%.0f+%.0f) in %s\n\n" a !ground_alt !security_height (Xml.to_string x)
  end

let print_waypoint = fun rel_utm_of_wgs84 default_alt waypoint ->
  let (x, y) =
    try
      rel_utm_of_wgs84 {posn_lat=(Deg>>Rad)(float_attrib waypoint "lat");
			posn_long=(Deg>>Rad)(float_attrib waypoint "lon") }
    with
      Xml.No_attribute "lat" | Xml.No_attribute "lon" ->
	(float_attrib waypoint "x", float_attrib waypoint "y")
  and alt = try Xml.attrib waypoint "alt" with _ -> default_alt in
  check_altitude (float_of_string alt) waypoint;
  printf " {%.1f, %.1f, %s},\\\n" x y alt


let blocks = Hashtbl.create 11
let index_of_blocks = ref []
let stages = Hashtbl.create 11

let get_index_block = fun x ->
  try
    string_of_int (List.assoc x !index_of_blocks)
  with
    Not_found -> failwith (sprintf "Unknown block: '%s'" x)

let print_exception = fun x ->
  let i = get_index_block (ExtXml.attrib x "deroute") in
  let c = parsed_attrib x "cond" in
  lprintf "if %s { GotoBlock(%s) }\n" c i

let return_from_excpt l = Xml.Element ("return_from_excpt", ["name",l], [])
let goto l = Xml.Element ("goto", ["name",l], [])
let exit_block = Xml.Element ("exit_block", [], [])
  
let stage = ref 0

let output_label l = lprintf "Label(%s)\n" l

let output_vmode x wp last_wp =
  let pitch = try parsed_attrib x "pitch" with _ -> "0.0" in
  lprintf "nav_pitch = %s;\n" pitch;
  let vmode = try ExtXml.attrib x "vmode" with _ -> "alt" in
  begin
    match vmode with
      "climb" ->
	lprintf "vertical_mode = VERTICAL_MODE_AUTO_CLIMB;\n";
	lprintf "desired_climb = %s;\n" (parsed_attrib x "climb")
    | "alt" ->
	lprintf "vertical_mode = VERTICAL_MODE_AUTO_ALT;\n";
	let alt =
	  try
	    let a = parsed_attrib x "alt" in
	    begin
	      try
		check_altitude (float_of_string a) x
	      with
		Failure "float_of_string" -> ()
	    end;
	    a
	  with _ ->
	    if wp = "" then failwith "alt or waypoint required in alt vmode" else
	    sprintf "waypoints[%s].a" wp in
	lprintf "desired_altitude = %s;\n" alt;
	lprintf "pre_climb = 0.;\n"
    | "glide" ->
	lprintf "vertical_mode = VERTICAL_MODE_AUTO_ALT;\n";
	lprintf "glide_to(%s, %s);\n" last_wp wp
    | "gaz" ->
	lprintf "vertical_mode = VERTICAL_MODE_AUTO_GAZ;\n";
	lprintf "nav_desired_gaz = TRIM_UPPRZ(%s*MAX_PPRZ);\n" (parsed_attrib x "gaz")
    | x -> failwith (sprintf "Unknown vmode '%s'" x)
  end;
  vmode
	  
let output_hmode x wp last_wp =
  try
    let hmode = ExtXml.attrib x "hmode" in
    begin
      match hmode with
	"route" ->
	  if last_wp = "last_wp" then
	    fprintf stderr "WARNING: Deprecated use of 'route' using last waypoint in %s\n"(Xml.to_string x);
	  lprintf "route_to(%s, %s);\n" last_wp wp
      | "direct" -> lprintf "fly_to(%s);\n" wp
      | x -> failwith (sprintf "Unknown hmode '%s'" x)
    end;
    hmode
  with
    ExtXml.Error _ -> lprintf "fly_to(%s);\n" wp; "direct" (* Default behaviour *)
	  

let get_index_waypoint = fun x l ->
  try
    string_of_int (List.assoc x l)
  with
    Not_found -> failwith (sprintf "Unknown waypoint: %s\n" x)

	
let rec compile_stage = fun block x ->
  incr stage;
  Hashtbl.add stages x (block, !stage);
  begin
    match Xml.tag x with
     "while" ->
       List.iter (compile_stage block) (Xml.children x);
       incr stage (* To count the loop stage *)
    | "return_from_excpt" | "goto"  | "deroute" | "exit_block"
    | "heading" | "go" | "stay" | "xyz" | "circle" -> ()
    | s -> failwith (sprintf "Unknown stage: %s\n" s)
  end

let rec print_stage = fun index_of_waypoints x ->
  incr stage;
  let stage () = lprintf "Stage(%d)\n" !stage; right () in
  begin
    match String.lowercase (Xml.tag x) with
      "return_from_excpt" ->
	stage ();
	lprintf "ReturnFromException(%s)\n" (name_of x)
    | "goto" ->
	stage ();
	lprintf "Goto(%s)\n" (name_of x)
    | "deroute" ->
	stage ();
	lprintf "GotoBlock(%s)\n" (get_index_block (ExtXml.attrib x "block"))
    | "exit_block" ->
	stage ();
	lprintf "NextBlock()\n"
    | "while" ->
	let w = gen_label "while" in
	let e = gen_label "endwhile" in
	output_label w;
	stage ();
	let c = try parsed_attrib x "cond" with _ -> "TRUE" in
	lprintf "if (! (%s)) Goto(%s) else NextStage();\n" c e;
	List.iter (print_stage index_of_waypoints) (Xml.children x);
	print_stage index_of_waypoints (goto w);
	output_label e
    | "heading" ->
	stage ();
	let until = parsed_attrib x "until" in
	lprintf "if (%s) NextStage() else {\n" until;
	right (); 
	lprintf "desired_course = RadOfDeg(%s);\n" (parsed_attrib x "course");
	ignore (output_vmode x "" "");
	left (); lprintf "}\n";
	lprintf "return;\n"
    | "go" ->
	stage ();
	let wp = get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints in
	lprintf "if (approaching(%s)) NextStageFrom(%s) else {\n" wp wp;
	right ();
	let last_wp =
	  try
	    get_index_waypoint (ExtXml.attrib x "from") index_of_waypoints
	  with _ -> "last_wp" in
	let hmode = output_hmode x wp last_wp in
	let vmode = output_vmode x wp last_wp in
	if vmode = "glide" && hmode <> "route" then
	  failwith "glide vmode requires route hmode";
	left (); lprintf "}\n";
	lprintf "return;\n"
    | "stay" ->
	stage ();
	begin
	  try
	    let wp = get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints in
	    ignore(output_hmode x wp "");
	    ignore (output_vmode x wp "");
	  with
	    Xml2h.Error _ ->
	      lprintf "fly_to_xy(last_x, last_y);\n";
		ignore(output_vmode x "" "")
	end;
	lprintf "return;\n"
    | "xyz" ->
	stage ();
	let r = try parsed_attrib  x "radius" with _ -> "100" in
	lprintf "Goto3D(%s)\n" r;
	lprintf "return;\n"
    | "circle" ->
	stage ();
	let wp = get_index_waypoint (ExtXml.attrib x "wp") index_of_waypoints in
	let r = parsed_attrib  x "radius" in
	let vmode = output_vmode x wp "" in
	lprintf "Circle(%s, %s);\n" wp r;
	begin
	  try
	    let c = parsed_attrib x "until" in
	    lprintf "if (%s) NextStage();\n" c
	  with
	    ExtXml.Error _ -> ()
	end;
	lprintf "return;\n"
    | s -> failwith "Unreachable"
  end;
  left ()

let compile_block = fun block_num (b:Xml.xml) ->
  Hashtbl.add blocks b block_num;
  index_of_blocks := (name_of b, block_num) :: !index_of_blocks;
  stage := (-1);
  let stages =
    List.filter (fun x -> Xml.tag x <> "exception") (Xml.children b) in

  List.iter (compile_stage block_num) stages;

  compile_stage block_num exit_block

let compile_blocks = fun bs ->
  let block = ref (-1) in
  List.iter
    (fun b ->
      incr block;
      compile_block !block b)
    bs



let print_block = fun index_of_waypoints (b:Xml.xml) block_num ->
  let n = name_of b in
  lprintf "Block(%d) // %s\n" block_num n;

  let excpts, stages =
    List.partition (fun x -> Xml.tag x = "exception") (Xml.children b) in

  List.iter print_exception excpts;

  lprintf "switch(nav_stage) {\n";
  right ();
  stage := (-1);
  List.iter (print_stage index_of_waypoints) stages;

  print_stage index_of_waypoints exit_block;

  left (); lprintf "}\n\n"



let print_blocks = fun index_of_waypoints bs ->
  lprintf "#ifdef NAV_C\n";
  lprintf "\nstatic inline void auto_nav(void) {\n";
  right ();
  lprintf "switch (nav_block) {\n";
  right ();
  let block = ref (-1) in
  List.iter (fun b -> incr block; print_block index_of_waypoints b !block) bs;
  left (); lprintf "}\n";
  left (); lprintf "}\n";
  lprintf "#endif // NAV_C\n"


let define_home = fun waypoints ->
  let rec loop i = function
      [] -> failwith "Waypoint 'HOME' required"
    | w::ws ->
	if name_of w = "HOME" then begin
	  Xml2h.define "WP_HOME" (string_of_int i);
	  (float_attrib w "x", float_attrib w "y")
	end else
	  loop (i+1) ws in
  loop 0 waypoints

let check_distance = fun (hx, hy) max_d wp ->
  let x = float_attrib wp "x"
  and y = float_attrib wp "y" in
  let d = sqrt ((x-.hx)**2. +. (y-.hy)**2.) in
  if d > max_d then
    fprintf stderr "\nWARNING: Waypoint '%s' too far from HOME (%.0f>%.0f)\n\n" (name_of wp) d max_d
  


let _ =
  let xml_file = ref "fligh_plan.xml"
  and dump = ref false in
  Arg.parse [("-dump", Arg.Set dump, "Dump compile result");
	     ("-nocheck", Arg.Clear check_expressions, "Disable expression checking")]
    (fun f -> xml_file := f)
    "Usage:";
  try
    let xml = Xml.parse_file !xml_file in
    let dir = Filename.dirname !xml_file in
    let xml = Fp_proc.process_includes dir xml in
    (*** prerr_endline (Xml.to_string_fmt xml); prerr_endline "\n\n\n"; ***)
    let xml = Fp_proc.process_relative_waypoints xml in
    let waypoints = ExtXml.child xml "waypoints"
    and blocks = Xml.children (ExtXml.child xml "blocks") in

    compile_blocks blocks;

    if !dump then
      let block_names = List.map (fun (x,y) -> (y, x)) !index_of_blocks in
      let lstages = ref [] in
      Hashtbl.iter 
	(fun xml (b,s) ->
	  lstages :=
	    Xml.Element ("stage", [ "block", string_of_int b;
				    "block_name", List.assoc b block_names;
				    "stage", string_of_int s], [xml])
	    :: !lstages)
	stages;
      let xml_stages = Xml.Element ("stages", [], !lstages) in
      let dump_xml = Xml.Element ("dump", [], [xml; xml_stages]) in
      printf "%s\n" (ExtXml.to_string_fmt dump_xml)
    else begin
      let h_name = "FLIGHT_PLAN_H" in
      printf "/* This file has been generated from %s */\n" !xml_file;
      printf "/* Please DO NOT EDIT */\n\n";
      
      printf "#ifndef %s\n" h_name;
      Xml2h.define h_name "";
      printf "\n";
      

      let name = ExtXml.attrib xml "name" in
      Xml2h.warning ("FLIGHT PLAN: "^name);
      Xml2h.define_string "FLIGHT_PLAN_NAME" name;
      
      let get_float = fun x -> float_attrib xml x in
      let lat0_deg = get_float "lat0"
      and lon0_deg = get_float "lon0"
      and qfu = get_float "qfu"
      and mdfh = get_float "max_dist_from_home"
      and alt = ExtXml.attrib xml "alt" in
      security_height := get_float "security_height";
      ground_alt := get_float "ground_alt";

      check_altitude (float_of_string alt) xml;

      let utm0 = utm_of WGS84 {posn_lat=(Deg>>Rad)lat0_deg;posn_long=(Deg>>Rad)lon0_deg } in
      let rel_utm_of_wgs84 = fun wgs84 ->
	let utm = utm_of WGS84 wgs84 in
	(utm.utm_x -. utm0.utm_x, utm.utm_y -. utm0.utm_y) in

      Xml2h.define "NAV_UTM_EAST0" (sprintf "%.0f" utm0.utm_x);
      Xml2h.define "NAV_UTM_NORTH0" (sprintf "%.0f" utm0.utm_y);
      Xml2h.define "QFU" (sprintf "%.1f" qfu);

      
      let waypoints = Xml.children waypoints in
      let (hx, hy) = define_home waypoints in
      List.iter (check_distance (hx, hy) mdfh) waypoints;

      Xml2h.define "WAYPOINTS" "{ \\";
      List.iter (print_waypoint rel_utm_of_wgs84 alt) waypoints;
      lprintf "};\n";
      Xml2h.define "NB_WAYPOINT" (string_of_int (List.length waypoints));

      Xml2h.define "GROUND_ALT" (string_of_float !ground_alt);
      Xml2h.define "SECURITY_ALT" (string_of_float (!security_height +. !ground_alt));
      Xml2h.define "MAX_DIST_FROM_HOME" (string_of_float mdfh);
      
      let index_of_waypoints =
	let i = ref (-1) in
	List.map (fun w -> incr i; (name_of w, !i)) waypoints in

      print_blocks index_of_waypoints blocks;

      Xml2h.finish h_name
    end
  with
    Xml.Error e -> prerr_endline (Xml.error e); exit 1
  | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
