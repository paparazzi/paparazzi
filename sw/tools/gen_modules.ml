(*
 * $Id$
 *
 * XML preprocessing for modules
 *  
 * Copyright (C) 2009 Gautier Hattenberger
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

(** Default main frequency = 60Hz *)
let freq = ref 60

(** Formatting with a margin *)
let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let out_h = stdout

let lprintf = fun out f ->
  fprintf out "%s" (String.make !margin ' ');
  fprintf out f

let print_headers = fun modules ->
  lprintf out_h  "#include \"std.h\"\n";
  List.iter (fun m ->
    try
      let headers = ExtXml.child m "header" in
      List.iter (fun h ->
        lprintf out_h "#include \"%s\"\n" (Xml.attrib h "name"))
      (Xml.children headers)
    with _ -> ())
  modules

let get_status_name = fun f n ->
  let func = (Xml.attrib f "fun") in
  n^"_"^String.sub func 0 (try String.index func '(' with _ -> (String.length func))^"_status"

let print_status = fun modules out_c ->
  nl ();
  List.iter (fun m ->
    let module_name = ExtXml.attrib m "name" in
    List.iter (fun i ->
      match Xml.tag i with
        "periodic" ->
          lprintf out_h "extern uint8_t %s;\n" (get_status_name i module_name);
          fprintf out_c "uint8_t %s;\n" (get_status_name i module_name)
      | _ -> ())
    (Xml.children m))
  modules

let print_init_functions = fun modules ->
  lprintf out_h "\nstatic inline void modules_init(void) {\n";
  right ();
  List.iter (fun m ->
    let module_name = ExtXml.attrib m "name" in
    List.iter (fun i ->
      match Xml.tag i with
        "init" -> lprintf out_h "%s;\n" (Xml.attrib i "fun")
      | "periodic" -> lprintf out_h "%s = %s;\n" (get_status_name i module_name)
      (try Xml.attrib i "autorun" with _ -> "TRUE")
      | _ -> ())
    (Xml.children m))
  modules;
  left ();
  lprintf out_h "}\n"

let remove_dup = fun l ->
  let rec loop = fun l ->
    match l with
      [] | [_] -> l
    | x::((x'::_) as xs) ->
	if x = x' then loop xs else x::loop xs in
  loop (List.sort compare l)

let print_periodic_functions = fun modules ->
  let min_period = 1. /. float !freq in
  let max_period = 65536. /. float !freq in

  lprintf out_h "\nstatic inline void modules_periodic_task(void) {\n";
  right ();
  (** Computes the required modulos *)
  let functions_modulo = List.flatten (List.map (fun m ->
    let periodic = List.filter (fun i -> (String.compare (Xml.tag i) "periodic") == 0) (Xml.children m) in
    let module_name = ExtXml.attrib m "name" in
    List.map (fun x -> 
      let p = float_of_string (ExtXml.attrib x "period") in
      if p < min_period || p > max_period then
        fprintf stderr "Warning: period is bound between %.3fs and %.3fs for function %s\n%!" min_period max_period (ExtXml.attrib x "fun");
      ((x, module_name), min 65535 (max 1 (int_of_float (p*.float_of_int !freq)))))
    periodic) modules) in
  let modulos = remove_dup (List.map snd functions_modulo) in
  (** Print modulos *)
  List.iter (fun modulo ->
    let v = sprintf "i%d" modulo in
    let _type = if modulo >= 256 then "uint16_t" else "uint8_t" in
    lprintf out_h "static %s %s; %s++; if (%s>=%d) %s=0;\n" _type v v v modulo v;)
  modulos;
  (** Print start and stop functions *)
  List.iter (fun m ->
    let module_name = ExtXml.attrib m "name" in
    let periodic = List.filter (fun i -> (String.compare (Xml.tag i) "periodic") == 0) (Xml.children m) in
    nl ();
    List.iter (fun f ->
      let status = get_status_name f module_name in
      let start = (ExtXml.attrib_or_default f "start" "") in
      lprintf out_h "if (%s == MODULES_START) { %s; %s = MODULES_RUN; }\n" status start status;
      let stop = (ExtXml.attrib_or_default f "stop" "") in
      lprintf out_h "if (%s == MODULES_STOP) { %s; %s = MODULES_IDLE; }\n" status stop status;
      )
    periodic)
  modules;
  (** Print periodic functions *)
  let functions = List.sort (fun (_,p) (_,p') -> compare p p') functions_modulo in
  let i = ref 0 in (** Basic balancing:1 function every 10Hz FIXME *)
  let l = ref [] in
  nl ();
  let test_delay = fun x -> try let _ = Xml.attrib x "delay" in true with _ -> false in
  List.iter (fun ((func, name), p) ->
    let function_name = ExtXml.attrib func "fun" in
    if (test_delay func) then begin
      (** Delay is set by user *)
      let delay = int_of_string (Xml.attrib func "delay") in
      if delay >= p then fprintf stderr "Warning: delay is bound between 0 and %d for function %s\n" (p-1) function_name;
      let delay_p = delay mod p in
      let else_ = if List.mem_assoc p !l && not (List.mem (p, delay_p) !l) then
        "else " else "" in
      lprintf out_h "%sif (i%d == %d && %s == MODULES_RUN) {\n" else_ p delay_p (get_status_name func name);
      l := (p, delay_p) :: !l;
    end
    else begin
      (** Delay is automtically set *)
      i := !i mod p;
      let else_ = if List.mem_assoc p !l && not (List.mem (p, !i) !l) then "else " else "" in
      lprintf out_h "%sif (i%d == %d && %s == MODULES_RUN) {\n" else_ p !i (get_status_name func name);
      l := (p, !i) :: !l;
      let incr = p / ((List.length (List.filter (fun (_,p') -> compare p p' == 0) functions)) + 1) in
      i := !i + incr;
    end;
    right ();
    lprintf out_h "%s;\n" function_name;
    left ();
    lprintf out_h "}\n")
  functions;
  left ();
  lprintf out_h "}\n"

let print_event_functions = fun modules ->
  lprintf out_h "\nstatic inline void modules_event_task(void) {\n";
  right ();
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
        "event" -> lprintf out_h "%s;\n" (Xml.attrib i "fun")
      | _ -> ())
    (Xml.children m))
  modules;
  left ();
  lprintf out_h "}\n"


let parse_modules modules out_c =
  print_headers modules;
  print_status modules out_c;
  nl ();
  fprintf out_h "#ifdef MODULES_C\n";
  print_init_functions modules;
  print_periodic_functions modules;
  print_event_functions modules;
  nl ();
  fprintf out_h "#endif // MODULES_C\n"

let print_empty_functions = fun () ->
  fprintf out_h "#ifdef MODULES_C\n";
  fprintf out_h "\nstatic inline void modules_init(void) {}\n";
  fprintf out_h "\nstatic inline void modules_periodic_task(void) {}\n";
  fprintf out_h "\nstatic inline void modules_event_task(void) {}\n";
  fprintf out_h "\n#endif // MODULES_C\n"

let get_modules = fun dir m ->
  match Xml.tag m with
    "load" -> begin
      let name = ExtXml.attrib m "name" in
      let xml = Xml.parse_file (dir^name) in
      xml
		end
  | _ -> xml_error "load"

let test_section_modules = fun xml ->
  List.fold_right (fun x r -> ExtXml.tag_is x "modules" || r) (Xml.children xml) false 

(** Check dependencies *)
let pipe_regexp = Str.regexp "|"
let dep_of_field = fun field att ->
  try 
    Str.split pipe_regexp (Xml.attrib field att)
  with
    _ -> []

let check_dependencies = fun modules names ->
  List.iter (fun m ->
    try
      let dep = ExtXml.child m "depend" in
      let require = dep_of_field dep "require" in
      List.iter (fun req ->
        if not (List.exists (fun c -> String.compare c req == 0) names) then
          fprintf stderr "\nWARNING: Dependency not satisfied: module %s requires %s\n" (Xml.attrib m "name") req)
      require;
      let conflict = dep_of_field dep "conflict" in
      List.iter (fun con ->
        if List.exists (fun c -> String.compare c con == 0) names then
          fprintf stderr "\nWARNING: Dependency not satisfied: module %s conflicts with %s\n" (Xml.attrib m "name") con)
      conflict
    with _ -> ()
  ) modules

let h_name = "MODULES_H"

let () =
  if Array.length Sys.argv <> 4 then
    failwith (Printf.sprintf "Usage: %s conf_modules_dir out_c_file xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(3)
  and out_c = open_out Sys.argv.(2)
  and modules_dir = Sys.argv.(1) in
  try
    let xml = start_and_begin xml_file h_name in
    begin_c_out xml_file "modules" out_c;
    fprintf out_h "#ifdef USE_MODULES\n\n";
    fprintf out_c "#ifdef USE_MODULES\n\n";
    fprintf out_h "#define MODULES_IDLE  0\n";
    fprintf out_h "#define MODULES_RUN   1\n";
    fprintf out_h "#define MODULES_START 2\n";
    fprintf out_h "#define MODULES_STOP  3\n";
    nl ();
    if (test_section_modules xml) then begin
      let modules = (ExtXml.child xml "modules") in
      try
        let main_freq = Xml.attrib modules "main_freq" in
        freq := int_of_string main_freq;
      with _ -> ();
      let modules_list = List.map (get_modules modules_dir) (Xml.children modules) in
      let modules_name = 
        (List.map (fun l -> try Xml.attrib l "name" with _ -> "") (Xml.children modules)) @
        (List.map (fun m -> try Xml.attrib m "name" with _ -> "") modules_list) in
      check_dependencies modules_list modules_name;
      parse_modules modules_list out_c;
    end
    else print_empty_functions ();
    fprintf out_h "\n#endif // USE_MODULES\n";
    fprintf out_c "\n#endif // USE_MODULES\n";
    finish h_name;
    close_out out_c;
  with
    Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
  | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
  | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
  | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
