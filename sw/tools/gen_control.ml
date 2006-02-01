(*
 * $Id$
 *
 * XML preprocessing for airframe parameters
 *  
 * Copyright (C) 2006 Antoine Drouin
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
open Xml2h

let fos = float_of_string


type loop_type = P | PI | PD | PID
let loop_type_of_string = fun s ->
  match s with
    "P" -> P
  | "PI" -> PI
  | "PD" -> PD
  | "PID" -> PID
  | s -> invalid_arg (sprintf "loop_type: %s" s)
let is_i = function (PI | PID) -> true | _ -> false
let is_d = function (PD | PID) -> true | _ -> false
let is_p = fun lt -> lt = P
let is_pd = fun lt -> lt = PD
let is_pid = fun lt -> lt = PID

type loop = { 
    name : string; 
    loop_type : loop_type; 
    pgain : float; 
    dgain : float; 
    igain : float; 
    isat : string;
    data_type : string;  
    measure : string; 
    setpoint : string; 
    output : string;
    osat : string;
  }

type mode = C | H

let mode_of_string = function
    "c" -> C
  | "h" -> H
  | s -> invalid_arg (sprintf "mode_of_string: %s" s)

let print_mode_inputs = fun s ->
  match Xml.tag s with
    "input" ->
      let input = ExtXml.attrib s "input"
      and output = ExtXml.attrib s "output" 
      and range =  ExtXml.attrib s "range" in
      printf "  %s = %s * %s;\n" output input range
  | _ -> ()

let print_mode_loops = fun s ->
  match Xml.tag s with
    "run" ->
      let name = ExtXml.attrib s "name" in
      printf "  control_run_%s_loops();\n" name
  | _ -> ()

let parse_loop = fun s list ->
  match Xml.tag s with
    "loop" ->
      let na = ExtXml.attrib s "name"
      and lt = loop_type_of_string (ExtXml.attrib s "loop_type")
      and pg = fos (ExtXml.attrib s "pgain")
      and dg = fos (ExtXml.attrib_or_default s "dgain" "0")
      and ig = fos (ExtXml.attrib_or_default s "igain" "0")
      and is = ExtXml.attrib_or_default s "integral_saturation" "0"
      and dt = ExtXml.attrib s "data_type" 
      and mea = ExtXml.attrib s "measure" 
      and sp = ExtXml.attrib s "setpoint" 
      and op = ExtXml.attrib s "output" 
      and os = ExtXml.attrib s "saturation" in
      let param = {name = na; loop_type = lt; 
		   pgain = pg; dgain = dg; igain = ig; isat = is; 
		   data_type = dt; measure = mea; setpoint = sp; output = op; osat = os} in
      param::list
  | _ -> list


let declare = fun mode _type name value ->
  match mode with
    H ->
      printf "extern %s %s;\n" _type name
  | C ->
      printf "%s %s = %s;\n" _type name value
	
	
let print_loop_declaration = fun mode lp ->
  declare mode lp.data_type ("control_"^lp.name^"_setpoint") "0";
  declare mode "float" ("control_"^lp.name^"_pgain") (string_of_float lp.pgain);
  if is_d lp.loop_type then
    begin
      declare mode lp.data_type  ("control_"^lp.name^"_last_err") "0";
      declare mode "float" ("control_"^lp.name^"_dgain") (string_of_float lp.dgain);
    end;
  if is_i lp.loop_type then
    begin
      declare mode lp.data_type  ("control_"^lp.name^"_sum_err") "0";
      declare mode "float" ("control_"^lp.name^"_igain") (string_of_float lp.igain);
    end;
  nl()

let print_loop_code = fun mode lp ->
  printf "  {\n";
  printf "    %s err = %s - %s;\n" lp.data_type lp.measure lp.setpoint;
  if is_d lp.loop_type then
    begin
      printf "    %s d_err = err - control_%s_last_err;\n" lp.data_type lp.name;
      printf "    control_%s_last_err = err;\n" lp.name
    end;
  if is_i lp.loop_type then
      printf "    control_%s_sum_err += err;\n" lp.name;
  if is_p lp.loop_type then
    printf "    %s = ChopAbs(%f * err, %s);\n" lp.output lp.pgain lp.osat;
  if is_pd lp.loop_type then
    printf "    %s = ChopAbs(%f * (err + %f * d_err), %s);\n" lp.output lp.pgain lp.dgain lp.osat;
  if is_pid lp.loop_type then
    printf "    %s =  ChopAbs(%f * (err + %f * d_err + %f * control_%s_sum_err), %s);\n" lp.output lp.pgain lp.dgain lp.igain lp.name lp.osat;
  printf "  }\n"

let parse_control = fun mode s ->
  match Xml.tag s with
    "level" ->
      let loops_params = List.fold_right parse_loop (Xml.children s) [] in
      List.iter (print_loop_declaration mode) loops_params;
      begin
	match mode with
	  H ->
	    nl();
	    let level_name = ExtXml.attrib s "name" in
	    printf "static inline void control_run_%s_loops ( void ) {\n" level_name;
	    List.iter (print_loop_code mode) loops_params;
	    printf "}\n"
	| C -> ()
      end;
      nl()
  | "mode" ->
      begin
	match mode with
	  H ->
	    let mode_name = ExtXml.attrib s "name" in
	    printf "static inline void control_process_radio_control_%s ( void ) {\n" mode_name;
	    List.iter print_mode_inputs (Xml.children s);
	    printf "}\n";
	    nl();
	    printf "static inline void control_run_%s ( void ) {\n" mode_name;
	    List.iter print_mode_loops (Xml.children s);
	    printf "}\n";
	    nl()
	| C -> ()
      end
  | t -> failwith (sprintf "Unexpected tag: %s" t)
	
let parse_section = fun mode s ->
  match Xml.tag s with
    "control" ->
      List.iter (parse_control mode) (Xml.children s)
  | _ -> ()


let h_name = "CONTROL_H"
let c_name = "control"
 
let _ =
  if Array.length Sys.argv <> 3 then
    failwith (Printf.sprintf "Usage: %s [c/h] xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(2) in
  let mode = mode_of_string Sys.argv.(1) in
  try
    begin
      match mode with
	H ->
	  let xml = start_and_begin xml_file h_name in
	  printf "#include \"std.h\"\n";
	  printf "#include ESTIMATOR\n";
	  printf "#include \"airframe.h\"\n";
	  printf "#include \"radio_control.h\"\n";
	  printf "#include \"paparazzi.h\"\n";
	  nl();
	  declare mode "pprz_t" "control_commands[COMMANDS_NB]" "";
	  nl();
	  List.iter (parse_section mode) (Xml.children xml);
	  finish h_name
      | C -> 
	  let xml = start_and_begin_c xml_file c_name in
	  declare mode "pprz_t" "control_commands[COMMANDS_NB]" "COMMANDS_FAILSAFE";
	  List.iter (parse_section mode) (Xml.children xml);
    end
  with
    Xml.Error e -> prerr_endline (Xml.error e)

