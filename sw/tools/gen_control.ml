open Printf
open Xml2h

let fos = float_of_string

type loop = { 
    name : string; 
    loop_type : string; 
    pgain : float; 
    dgain : float; 
    igain : float; 
    isat : string;
    data_type : string;  
    measure : string; 
    setpoint : string; 
    output : string;
    sat : string;
  }

let parse_input = fun s ->
  match Xml.tag s with
    "input" ->
      let input = ExtXml.attrib s "input"
      and output = ExtXml.attrib s "output" 
      and range =  ExtXml.attrib s "range" in
      printf "  %s = %s * %s;\n" output input range
  | _ -> ignore ()

let parse_loop = fun s list ->
  match Xml.tag s with
    "loop" ->
      let na = ExtXml.attrib s "name"
      and lt = ExtXml.attrib s "loop_type"
      and pg = fos (ExtXml.attrib s "pgain")
      and dg = fos (ExtXml.attrib_or_default s "dgain" "0")
      and ig = fos (ExtXml.attrib_or_default s "igain" "0")
      and is = ExtXml.attrib_or_default s "integral_saturation" "0"
      and dt = ExtXml.attrib s "data_type" 
      and mea = ExtXml.attrib s "measure" 
      and sp = ExtXml.attrib s "setpoint" 
      and op = ExtXml.attrib s "output" 
      and sa = ExtXml.attrib s "saturation" in
      let param = {name = na; loop_type = lt; 
		   pgain = pg; dgain = dg; igain = ig; isat = is; 
		   data_type = dt; measure = mea; setpoint = sp; output = op; sat = sa} in
      param::list
  | _ -> list


let print_loop_declaration = fun lp ->
  printf "static extern %s control_%s_setpoint = 0;\n" lp.data_type lp.name;
  printf "static extern float control_%s_pgain = %f;\n" lp.name lp.pgain;
  if Str.string_match (Str.regexp ".*D.*") lp.loop_type 0 then
    begin
      printf "static extern %s control_%s_last_err = %f;\n" lp.data_type lp.name lp.dgain;
      printf "static extern float control_%s_dgain = %f;\n" lp.name lp.dgain
    end;
  if Str.string_match (Str.regexp ".*I.*") lp.loop_type 0 then
    begin
      printf "static extern %s control_%s_sum_err = 0;\n" lp.data_type lp.name;
      printf "static extern float control_%s_igain = %f;\n" lp.name lp.igain
    end;
  nl()

let print_loop_code = fun lp ->
  printf "  {\n";
  printf "    %s err = %s - %s;\n" lp.data_type lp.measure lp.setpoint;
  if Str.string_match (Str.regexp ".*D.*") lp.loop_type 0 then
    begin
      printf "    %s d_err = err - control_%s_last_err;\n" lp.data_type lp.name;
      printf "    control_%s_last_err = err;\n" lp.name
    end;
  if Str.string_match (Str.regexp ".*I.*") lp.loop_type 0 then
      printf "    control_%s_sum_err += err;\n" lp.name;
  if Str.string_match (Str.regexp "^P$") lp.loop_type 0 then
    printf "    %s = ChopAbs(%f * err, %s);\n" lp.output lp.pgain lp.sat;
  if Str.string_match (Str.regexp "^PD$") lp.loop_type 0 then
    printf "    %s = ChopAbs(%f * (err + %f * d_err), %s);\n" lp.output lp.pgain lp.dgain lp.sat;
  if Str.string_match (Str.regexp "^PID$") lp.loop_type 0 then
    printf "    %s =  ChopAbs(%f * (err + %f * d_err + %f * control_%s_sum_err), %s);\n" lp.output lp.pgain lp.dgain lp.igain lp.name lp.sat;
  printf "  }\n"

let parse_control = fun s ->
  match Xml.tag s with
    "level" ->
      let loops_params = List.fold_right parse_loop (Xml.children s) [] in
      List.iter print_loop_declaration loops_params;
      nl();
      let level_name = ExtXml.attrib s "name" in
      printf "static inline void control_run_%s_loops ( void ) {\n" level_name;
      List.iter print_loop_code loops_params;
      printf "}\n";
      nl()
  | "mode" ->
      let mode_name = ExtXml.attrib s "name" in
      printf "static inline void control_process_radio_control_%s ( void ) {\n" mode_name;
      List.iter parse_input (Xml.children s);
      printf "}\n";
      nl()
  | _ -> ignore ()

let parse_section = fun s ->
  match Xml.tag s with
    "control" ->
      List.iter parse_control (Xml.children s)
  | _ -> ignore ()


let h_name = "CONTROL_H"
 
let _ =
  if Array.length Sys.argv <> 2 then
    failwith (Printf.sprintf "Usage: %s xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(1) in
  try
    let xml = start_and_begin xml_file h_name in
    printf "#include \"std.h\"\n";
    nl();
    List.iter parse_section (Xml.children xml);
    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
	  
