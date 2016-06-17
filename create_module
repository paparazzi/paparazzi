#!/usr/bin/ocamlrun /usr/bin/ocaml
#load "unix.cma";;
#directory "+xml-light";;
#load "xml-light.cma";;

open Printf;;
open Scanf;;

let ask = fun text ->
  printf "%s: " text;
  flush stdout;
  input_line stdin;;

type optional = Optional | Mandatory;;

let rec ask_param = fun text opt ->
  match opt with
  | Optional ->
      printf " - %s (optional): " text; flush stdout;
      input_line stdin;
  | Mandatory ->
      printf " - %s (mandatory): " text; flush stdout;
      let res = input_line stdin in
      if res = "" then ask_param text opt else res;
;;

let quit_with_error = fun text err ->
  printf "%s\n" text;
  exit err;;

printf "       ##########################\n";;
printf "       ##    Module Creator    ##\n";;
printf "       ##########################\n\n";;
printf "This program will help you to create a new module for Paparazzi\n";;
printf "Not all options are accessible from this program,\n";
printf "see http://wiki.paparazziuav.org/wiki/Modules for more information.\n";
(*printf "Please follow the instruction or pass a module xml file as intput\n\n";;*)
printf "Please follow the instruction\n\n";;
let name = String.lowercase (ask "Enter your module name");;
if name = "" then quit_with_error "module name must not be empty. Leaving." 1;;
let dir = String.lowercase (ask "Enter a module directory or leave blank if same as name");;
let desc = ask "Enter a short description of your module";;

let init_list = ref [];;
let periodic_list = ref [];;
let event_list = ref [];;
let datalink_list = ref [];;

let add_to_list = fun l a ->
  l := !l @ [a];;

let ask_init = fun () ->
  if (List.length !init_list) > 0 then
    printf "You already added an init function.\n"
  else begin
    printf "Initialization function to call, eg \"foo_init()\":\n";
    let name = String.lowercase (ask_param "function" Mandatory) in
    add_to_list init_list name
  end;
  true;;

let ask_periodic = fun () ->
  printf "Parameters for a periodic function:\n";
  let name = String.lowercase (ask_param "function name" Mandatory) in
  let freq = ask_param "call frequency" Mandatory in
  let start =  String.lowercase (ask_param "start function" Optional) in
  let stop =  String.lowercase (ask_param "stop function" Optional) in
  let auto = ask_param "autorun flag [TRUE, FALSE, LOCK (default)]" Optional in
  add_to_list periodic_list (name, freq, start, stop, auto);
  true;;

let ask_event = fun () ->
  printf "Parameters for an event function:\n";
  let name = String.lowercase(ask_param "function name" Mandatory) in
  add_to_list event_list name;
  true;;

let ask_datalink = fun () ->
  printf "Parameters for a datalink event callback:\n";
  let msg = ask_param "message name" Mandatory in
  let name = ask_param "callback function name" Mandatory in
  add_to_list datalink_list (msg, name);
  true;;

let show_module = fun () ->
  printf "\nModule name: %s\n" name;
  if dir <> "" then printf "Module directory: %s\n" dir;
  if desc <> "" then printf "Module description: %s\n" desc;
  List.iter (fun n -> printf "Init function: %s\n" n) !init_list;
  List.iter (fun (n, f, start, stop, auto) ->
    let _start = if start <> "" then " [ Start: "^start^" ] " else "" in
    let _stop = if stop <> "" then " [ Stop: "^stop^" ] " else "" in
    let _auto = if auto <> "" then " [ Auto: "^auto^" ] " else "" in
    printf "Periodic function: %s at %s Hz%s%s%s\n" n f _start _stop _auto;
  ) !periodic_list;
  List.iter (fun n -> printf "Event function: %s\n" n) !event_list;
  List.iter (fun (m, n) -> printf "Datalink event: %s on message %s\n" n m) !datalink_list;
  flush stdout;
  true;;

let choices = [
  ("Initialization function", ask_init);
  ("Periodic function", ask_periodic);
  ("Event function", ask_event);
  ("Datalink event function", ask_datalink);
  ("Display the current state of your module", show_module);
  ("End process and create module", (fun () -> false));
  ("Cancel and leave program", (fun () -> quit_with_error "Cancel by operator. Leaving." 2))
];;

let ask_choice = fun choices ->
  printf "\nPlease select the type of function to add to your module:\n";
  let rec display = fun i c ->
    match c with
    | (text, _) :: cl ->
        printf "%d - %s\n" i text;
        display (i+1) cl;
    | [] -> ()
  in
  display 1 choices;
  printf "Type number [1-%d]: " (List.length choices);
  flush stdout;
  try
    let r = int_of_string (input_line stdin) in
    let (_, f) = (Array.of_list choices).(r-1) in
    f ()
  with _ -> printf "Invalid entry\n"; true
;;

while ask_choice choices do () done;;

let xml_of_name = fun n d ->
  if d <> "" then [("name", n); ("dir", d)]
  else [("name", n)];;

let header_file = name^".h";;
let code_file = name^".c";;

let xml_header_of_name = fun n ->
  [Xml.Element ("header", [], [
    Xml.Element ("file", [("name", n)], [])
  ])];;

let xml_of_description = fun d ->
  let desc = Xml.Element ("description", [], [Xml.PCData d]) in
  [Xml.Element ("doc", [], [desc])];;

let xml_of_init = fun l ->
  List.map (fun n -> Xml.Element ("init", [("fun", n)], [])) l;;

let xml_of_peridic = fun l ->
  List.map (fun (n, f, start, stop, auto) ->
    let _start = if start <> "" then [("start", start)] else [] in
    let _stop = if stop <> "" then [("stop", stop)] else [] in
    let _auto = if auto <> "" then [("autorun", auto)] else [] in
    Xml.Element ("periodic", List.flatten [
      [("fun", n)]; [("freq", f)]; _start; _stop; _auto],
      [])
  ) l;;

let xml_of_event = fun l ->
  List.map (fun n -> Xml.Element ("event", [("fun", n)], [])) l;;

let xml_of_datalink = fun l ->
  List.map (fun (m, n) -> Xml.Element ("datalink", [("message", m); ("fun", n)], [])) l;;

let xml_makefile_of_name = fun n ->
  [Xml.Element ("makefile", [], [
    Xml.Element ("file", [("name", n)], [])
  ])];;

let module_xml = Xml.Element ("module", xml_of_name name dir, List.flatten [
  xml_of_description desc;
  xml_header_of_name header_file;
  xml_of_init !init_list;
  xml_of_peridic !periodic_list;
  xml_of_event !event_list;
  xml_of_datalink !datalink_list;
  xml_makefile_of_name code_file]);;

let (//) = Filename.concat;;

let dir_name = if dir = "" then name else dir;;
let xml_name = "conf/modules" // name^".xml";;
let header_name = "sw/airborne/modules"// dir_name // name^".h";;
let code_name = "sw/airborne/modules" // dir_name // name^".c";;

let test_filename = fun () ->
  let ask_confirm = ref false in
  let disp_msg = fun f c ->
    printf "File %s already exists\n" f;
    c := true;
  in
  if Sys.file_exists xml_name then disp_msg xml_name ask_confirm;
  if Sys.file_exists header_name then disp_msg header_name ask_confirm;
  if Sys.file_exists code_name then disp_msg code_name ask_confirm;
  if !ask_confirm then begin
    if String.lowercase (ask "Confirm erasing files ? [y,N]") <> "y"
    then quit_with_error "Not erasing existing files. Leaving" 1
  end;;

let write_module_xml = fun out ->
  fprintf out "<!DOCTYPE module SYSTEM \"module.dtd\">\n\n";
  fprintf out "%s\n\n" (Xml.to_string_fmt module_xml);;

let write_license = fun out ->
  fprintf out " * paparazzi is free software; you can redistribute it and/or modify\n";
  fprintf out " * it under the terms of the GNU General Public License as published by\n";
  fprintf out " * the Free Software Foundation; either version 2, or (at your option)\n";
  fprintf out " * any later version.\n";
  fprintf out " *\n";
  fprintf out " * paparazzi is distributed in the hope that it will be useful,\n";
  fprintf out " * but WITHOUT ANY WARRANTY; without even the implied warranty of\n";
  fprintf out " * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n";
  fprintf out " * GNU General Public License for more details.\n";
  fprintf out " *\n";
  fprintf out " * You should have received a copy of the GNU General Public License\n";
  fprintf out " * along with paparazzi; see the file COPYING.  If not, see\n";
  fprintf out " * <http://www.gnu.org/licenses/>.\n";;

let write_copyright = fun out author gpl ->
  fprintf out "/*\n";
  fprintf out " * Copyright (C) %s\n" author;
  fprintf out " *\n";
  fprintf out " * This file is part of paparazzi\n";
  fprintf out " *\n";
  if gpl then write_license out;
  fprintf out " */\n";;

let write_doxygen_header = fun out author filename description ->
  fprintf out "/**\n";
  fprintf out " * @file %s\n" filename;
  fprintf out " * @author %s\n" author;
  fprintf out " * %s\n" description;
  fprintf out " */\n";;

let write_header = fun out author gpl ->
  write_copyright out author gpl;
  write_doxygen_header out author (sprintf "\"modules/%s/%s.h\"" dir_name name) desc;
  fprintf out "\n#ifndef %s_H" (String.uppercase name);
  fprintf out "\n#define %s_H\n\n" (String.uppercase name);
  List.iter (fun f -> fprintf out "// extern void %s;\n" f) !init_list;
  List.iter (fun (f,_,_,_,_) -> fprintf out "// extern void %s;\n" f) !periodic_list;
  List.iter (fun f -> fprintf out "// extern void %s;\n" f) !event_list;
  List.iter (fun (_,f) -> fprintf out "// extern void %s;\n" f) !datalink_list;
  fprintf out "\n#endif\n\n";;

let write_code = fun out author gpl ->
  write_copyright out author gpl;
  write_doxygen_header out author (sprintf "\"modules/%s/%s.c\"" dir_name name) desc;
  fprintf out "\n#include \"modules/%s/%s.h\"\n\n" dir_name name;
  List.iter (fun f -> fprintf out "// void %s {}\n" f) !init_list;
  List.iter (fun (f,_,_,_,_) -> fprintf out "// void %s {}\n" f) !periodic_list;
  List.iter (fun f -> fprintf out "// void %s {}\n" f) !event_list;
  List.iter (fun (_,f) -> fprintf out "// void %s {}\n" f) !datalink_list;
  fprintf out "\n\n";;

print_newline ();;
test_filename ();;

print_newline ();;
let author = ask "Author name";;
let gpl = ask "Do you want to use GPLv2 license ? [Y,n]";;
let gpl = String.lowercase (gpl) = "y" || gpl = "";;

printf "Creating file %s ... " xml_name;;
try
  let xml_out = open_out xml_name in
  write_module_xml xml_out;
  close_out xml_out;
  printf "[done]\n";
with _ -> printf "[fail]\n";
flush stdout;;

Sys.command (sprintf "mkdir -p %s" ("sw/airborne/modules" // dir_name));;

printf "Creating file %s ... " header_name;;
try
  let h_out = open_out header_name in
  write_header h_out author gpl;
  close_out h_out;
  printf "[done]\n";
with _ -> printf "[fail]\n";
flush stdout;;

printf "Creating file %s ... " code_name;;
try
  let c_out = open_out code_name in
  write_code c_out author gpl;
  close_out c_out;
  printf "[done]\n";
with _ -> printf "[fail]\n";
flush stdout;;

printf "\nModule %s created\n" name;;
