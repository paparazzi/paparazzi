open Printf
open Xml2h

let parse_input = fun s ->
  match Xml.tag s with
    "input" ->
      let input = ExtXml.attrib s "input"
      and output = ExtXml.attrib s "output" in
      printf "  %s = %s;\n" output input 
  | _ -> ignore ()



let parse_loop = fun s ->
  match Xml.tag s with
    "loop" ->
      let name = ExtXml.attrib s "name"
      and loop_type = ExtXml.attrib s "type"
      and meas = ExtXml.attrib s "type" in
      printf "name %s\n" name
  | _ -> ignore ()



let parse_control = fun s ->
  match Xml.tag s with
    "level" ->
      let level_name = ExtXml.attrib s "name" in
      printf "static inline void control_run_%s_loops ( void ) {\n" level_name;
      List.iter parse_loop (Xml.children s);
      printf "}\n";
      nl();
  | "mode" ->
      let mode_name = ExtXml.attrib s "name" in
      printf "static inline void control_process_radio_control_%s ( void ) {\n" mode_name;
      List.iter parse_input (Xml.children s);
      printf "}\n";
      nl();
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
    List.iter parse_section (Xml.children xml);
    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
	  
