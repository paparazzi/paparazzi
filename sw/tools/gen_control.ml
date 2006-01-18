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
    osat : string;
  }

let mode = ref "c"


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
      and os = ExtXml.attrib s "saturation" in
      let param = {name = na; loop_type = lt; 
		   pgain = pg; dgain = dg; igain = ig; isat = is; 
		   data_type = dt; measure = mea; setpoint = sp; output = op; osat = os} in
      param::list
  | _ -> list


let declare = fun mode _type name value ->
  match mode with
    "h" ->
      printf "extern %s %s;\n" _type name
  | "c" ->
      printf "%s %s = %s;\n" _type name value
  | _ -> ignore()
	
	
let print_loop_declaration = fun lp ->
  declare !mode lp.data_type ("control_"^lp.name^"_setpoint") "0";
  declare !mode "float" ("control_"^lp.name^"_pgain") (string_of_float lp.pgain);
  if Str.string_match (Str.regexp ".*D.*") lp.loop_type 0 then
    begin
      declare !mode lp.data_type  ("control_"^lp.name^"_last_err") "0";
      declare !mode "float" ("control_"^lp.name^"_dgain") (string_of_float lp.dgain);
    end;
  if Str.string_match (Str.regexp ".*I.*") lp.loop_type 0 then
    begin
      declare !mode lp.data_type  ("control_"^lp.name^"_sum_err") "0";
      declare !mode "float" ("control_"^lp.name^"_igain") (string_of_float lp.igain);
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
    printf "    %s = ChopAbs(%f * err, %s);\n" lp.output lp.pgain lp.osat;
  if Str.string_match (Str.regexp "^PD$") lp.loop_type 0 then
    printf "    %s = ChopAbs(%f * (err + %f * d_err), %s);\n" lp.output lp.pgain lp.dgain lp.osat;
  if Str.string_match (Str.regexp "^PID$") lp.loop_type 0 then
    printf "    %s =  ChopAbs(%f * (err + %f * d_err + %f * control_%s_sum_err), %s);\n" lp.output lp.pgain lp.dgain lp.igain lp.name lp.osat;
  printf "  }\n"

let parse_control = fun s ->
  match Xml.tag s with
    "level" ->
      let loops_params = List.fold_right parse_loop (Xml.children s) [] in
      List.iter print_loop_declaration loops_params;
      begin
	match !mode with
	  "h" ->
	    nl();
	    let level_name = ExtXml.attrib s "name" in
	    printf "static inline void control_run_%s_loops ( void ) {\n" level_name;
	    List.iter print_loop_code loops_params;
	    printf "}\n"
	| _ -> ignore();
      end;
      nl()
  | "mode" ->
      begin
	match !mode with
	  "h" ->
	    let mode_name = ExtXml.attrib s "name" in
	    printf "static inline void control_process_radio_control_%s ( void ) {\n" mode_name;
	    List.iter parse_input (Xml.children s);
	    printf "}\n";
	    nl()
	| _ -> ignore();
      end
  | _ -> ignore ()
	
let parse_section = fun s ->
  match Xml.tag s with
    "control" ->
      List.iter parse_control (Xml.children s)
  | _ -> ignore ()


let h_name = "CONTROL_H"
let c_name = "control"
 
let _ =
  if Array.length Sys.argv <> 3 then
    failwith (Printf.sprintf "Usage: %s [c/h] xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(2) in
  mode := Sys.argv.(1);
  try
    begin
      match !mode with
	"h" ->
	  let xml = start_and_begin xml_file h_name in
	  printf "#include \"std.h\"\n";
	  printf "#include \"estimator.h\"\n";
	  printf "#include \"airframe.h\"\n";
	  printf "#include \"radio_control.h\"\n";
	  printf "#include \"paparazzi.h\"\n";
	  nl();
	  declare !mode "pprz_t" "control_commands[COMMANDS_NB]" "";
	  nl();
	  List.iter parse_section (Xml.children xml);
	  finish h_name
      | _ -> 
	  let xml = start_and_begin_c xml_file c_name in
	  declare !mode "pprz_t" "control_commands[COMMANDS_NB]" "COMMANDS_FAILSAFE";
	  List.iter parse_section (Xml.children xml);
    end
  with
    Xml.Error e -> prerr_endline (Xml.error e)

