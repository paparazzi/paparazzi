(*
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
module GC = Gen_common

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
    let dir_name = try Xml.attrib m "dir" with _ -> Xml.attrib m "name" in
    try
      let headers = ExtXml.child m "header" in
      List.iter (fun h ->
        let dir = ExtXml.attrib_or_default h "dir" dir_name in
        lprintf out_h "#include \"%s/%s\"\n" dir (Xml.attrib h "name"))
        (Xml.children headers)
    with _ -> ())
    modules

let get_status_name = fun f n ->
  let func = (Xml.attrib f "fun") in
  n^"_"^String.sub func 0 (try String.index func '(' with _ -> (String.length func))^"_status"

let get_status_shortname = fun f ->
  let func = (Xml.attrib f "fun") in
  String.sub func 0 (try String.index func '(' with _ -> (String.length func))

let get_period_and_freq = fun f max_freq ->
  let period = try Some (float_of_string (Xml.attrib f "period")) with _ -> None
  and freq = try Some (float_of_string (Xml.attrib f "freq")) with _ -> None in
  match period, freq with
    | None, None -> (1. /. max_freq, max_freq)
    | Some _p, None -> (_p, 1. /. _p)
    | None, Some _f -> (1. /. _f, _f)
    | Some _p, Some _ ->
      fprintf stderr "Warning: both period and freq are defined but only period is used for function %s\n" (ExtXml.attrib f "fun");
      (_p, 1. /. _p)

(* Extract function name and return in capital letters *)
let get_cap_name = fun f ->
  let name = Str.full_split (Str.regexp "[()]") f in
  match name with
    | [Str.Text t]
    | [Str.Text t; Str.Delim "("; Str.Delim ")"]
    | [Str.Text t; Str.Delim "("; Str.Text _ ; Str.Delim ")"] -> String.uppercase t
    | _ -> failwith "Gen_modules: not a valid function name"

let print_function_freq = fun modules ->
  let max_freq = float !freq in
  nl ();
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
          "periodic" ->
            let fname = get_cap_name (Xml.attrib i "fun") in
            let p, f = get_period_and_freq i max_freq in
            lprintf out_h "#define %s_PERIOD %f\n" fname p;
            lprintf out_h "#define %s_FREQ %f\n" fname f;
        | _ -> ())
      (Xml.children m))
    modules

let is_status_lock = fun p ->
  let mode = ExtXml.attrib_or_default p "autorun" "LOCK" in
  mode = "LOCK"

let print_status = fun modules ->
  nl ();
  List.iter (fun m ->
    let module_name = ExtXml.attrib m "name" in
    List.iter (fun i ->
      match Xml.tag i with
          "periodic" ->
            if not (is_status_lock i) then begin
              lprintf out_h "EXTERN_MODULES uint8_t %s;\n" (get_status_name i module_name);
            end
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
        | "periodic" -> if not (is_status_lock i) then
            lprintf out_h "%s = %s;\n" (get_status_name i module_name) (try match Xml.attrib i "autorun" with
                "TRUE" | "true" -> "MODULES_START"
              | "FALSE" | "false" | "LOCK" | "lock" -> "MODULES_IDLE"
              | _ -> failwith "Error: Unknown autorun value (possible values are: TRUE, FALSE, LOCK(default))"
              with _ -> "MODULES_IDLE" (* this should not be possible anyway *))
        | _ -> ())
      (Xml.children m))
    modules;
  left ();
  lprintf out_h "}\n"

let print_periodic_functions = fun modules ->
  let min_period = 1. /. float !freq
  and max_period = 65536. /. float !freq
  and min_freq   = float !freq /. 65536.
  and max_freq   = float !freq in

  lprintf out_h "\nstatic inline void modules_periodic_task(void) {\n";
  right ();
  (** Computes the required modulos *)
  let functions_modulo = List.flatten (List.map (fun m ->
    let periodic = List.filter (fun i -> (String.compare (Xml.tag i) "periodic") == 0) (Xml.children m) in
    let module_name = ExtXml.attrib m "name" in
    List.map (fun x ->
      let p, _ = get_period_and_freq x max_freq in
      if p < min_period || p > max_period then
        fprintf stderr "Warning: period is bound between %.3fs and %.3fs (%fHz and %.1fHz) for function %s\n%!"
          min_period max_period max_freq min_freq (ExtXml.attrib x "fun");
      ((x, module_name), min 65535 (max 1 (int_of_float (p *. float_of_int !freq))))
    ) periodic)
                                         modules) in
  let modulos = GC.singletonize (List.map snd functions_modulo) in
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
      if (is_status_lock f) then begin
        try lprintf out_h "%s;\n" (Xml.attrib f "start") with _ -> ();
          try let stop = Xml.attrib f "stop" in fprintf stderr "Warning: stop %s function will not be called\n" stop with _ -> ();
      end
      else begin
        let status = get_status_name f module_name in
        lprintf out_h "if (%s == MODULES_START) {\n" status;
        right ();
        ignore(try lprintf out_h "%s;\n" (Xml.attrib f "start") with _ -> ());
        lprintf out_h "%s = MODULES_RUN;\n" status;
        left ();
        lprintf out_h "}\n";
        lprintf out_h "if (%s == MODULES_STOP) {\n" status;
        right ();
        ignore(try lprintf out_h "%s;\n" (Xml.attrib f "stop") with _ -> ());
        lprintf out_h "%s = MODULES_IDLE;\n" status;
        left ();
        lprintf out_h "}\n";
      end
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
    if p = 1 then
      begin
        if (is_status_lock func) then
          lprintf out_h "%s;\n" function_name
        else begin
          lprintf out_h "if (%s == MODULES_RUN) {\n" (get_status_name func name);
          right ();
          lprintf out_h "%s;\n" function_name;
          left ();
          lprintf out_h "}\n";
        end
      end
    else
      begin
        if (test_delay func) then begin
          (** Delay is set by user *)
          let delay = int_of_string (Xml.attrib func "delay") in
          if delay >= p then fprintf stderr "Warning: delay is bound between 0 and %d for function %s\n" (p-1) function_name;
          let delay_p = delay mod p in
          let else_ = if List.mem_assoc p !l && not (List.mem (p, delay_p) !l) then
              "else " else "" in
          if (is_status_lock func) then
            lprintf out_h "%sif (i%d == %d) {\n" else_ p delay_p
          else
            lprintf out_h "%sif (i%d == %d && %s == MODULES_RUN) {\n" else_ p delay_p (get_status_name func name);
          l := (p, delay_p) :: !l;
        end
        else begin
          (** Delay is automtically set *)
          i := !i mod p;
          let else_ = if List.mem_assoc p !l && not (List.mem (p, !i) !l) then "else " else "" in
          if (is_status_lock func) then
            lprintf out_h "%sif (i%d == %d) {\n" else_ p !i
          else
            lprintf out_h "%sif (i%d == %d && %s == MODULES_RUN) {\n" else_ p !i (get_status_name func name);
          l := (p, !i) :: !l;
          let incr = p / ((List.length (List.filter (fun (_,p') -> compare p p' == 0) functions)) + 1) in
          i := !i + incr;
        end;
        right ();
        lprintf out_h "%s;\n" function_name;
        left ();
        lprintf out_h "}\n"
      end;
  ) functions;
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

let print_datalink_functions = fun modules ->
  lprintf out_h "\n#include \"pprzlink/messages.h\"\n";
  lprintf out_h "#include \"generated/airframe.h\"\n";
  lprintf out_h "static inline void modules_parse_datalink(uint8_t msg_id __attribute__ ((unused))) {\n";
  right ();
  let else_ = ref "" in
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
          "datalink" ->
            lprintf out_h "%sif (msg_id == DL_%s) { %s; }\n" !else_ (ExtXml.attrib i "message") (ExtXml.attrib i "fun");
            else_ := "else "
        | _ -> ())
      (Xml.children m))
    modules;
  left ();
  lprintf out_h "}\n"

let parse_modules modules =
  print_headers modules;
  print_function_freq modules;
  print_status modules;
  nl ();
  fprintf out_h "#ifdef MODULES_C\n";
  print_init_functions modules;
  print_periodic_functions modules;
  print_event_functions modules;
  nl ();
  fprintf out_h "#endif // MODULES_C\n";
  nl ();
  fprintf out_h "#ifdef MODULES_DATALINK_C\n";
  print_datalink_functions modules;
  nl ();
  fprintf out_h "#endif // MODULES_DATALINK_C\n"

let test_section_modules = fun xml ->
  List.fold_right (fun x r -> ExtXml.tag_is x "modules" || r) (Xml.children xml) false

(** create list of dependencies from string
 * returns a nested list, where the second level consists of OR dependencies
 *)
let deps_of_string = fun s ->
  let comma_regexp = Str.regexp "," in
  let pipe_regexp = Str.regexp "|" in
  try
    (* first get the comma separated deps *)
    let deps = Str.split comma_regexp s in
    (* split up each dependency in a list of OR deps (separated by |) *)
    List.map (fun dep ->
      Str.split pipe_regexp dep)
      deps;
  with
      _ -> [[]]

let get_pcdata = fun xml tag ->
  try
    Xml.pcdata (ExtXml.child (ExtXml.child xml tag) "0")
  with
      Not_found -> ""

(** Check dependencies *)
let check_dependencies = fun modules names ->
  List.iter (fun m ->
    try
      let module_name = Xml.attrib m "name" in
      let dep_string = get_pcdata m "depends" in
      (*fprintf stderr "\n\nWARNING: parsing dep string: %s\n\n" dep_string;
      fprintf stderr "\n\nWARNING: names: %s" (String.concat "," names);*)
      let require = deps_of_string dep_string in
      List.iter (fun deps ->
        (* iterate over all dependencies, where the second level contains the OR dependencies *)
        let find_common satisfied d = if List.mem d names then d::satisfied else satisfied in
        let satisfied = List.fold_left find_common [] deps in
        if List.length satisfied == 0 then
          begin
            fprintf stderr "\nDEPENDENCY WARNING: Module %s requires %s\n" module_name (String.concat " or " deps);
            fprintf stderr "Available dependencies are:\n    %s\n\n" (String.concat "\n    " names)
          end)
        require;
      let conflict_string = get_pcdata m "conflicts" in
      let conflict_l = List.flatten (deps_of_string conflict_string) in
      List.iter (fun con ->
        if List.exists (fun c -> String.compare c con == 0) names then
          fprintf stderr "\nDEPENDENCY WARNING: Module %s conflicts with %s\n" module_name con)
        conflict_l
    with _ -> ()
  ) modules

let write_settings = fun xml_file out_set modules ->
  fprintf out_set "<!-- This file has been generated by gen_modules from %s -->\n" xml_file;
  fprintf out_set "<!-- Version %s -->\n" (Env.get_paparazzi_version ());
  fprintf out_set "<!-- Please DO NOT EDIT -->\n\n";
  fprintf out_set "<settings>\n";
  fprintf out_set " <dl_settings>\n";
  let setting_exist = ref false in
  List.iter (fun m ->
    let module_name = ExtXml.attrib m "name" in
    List.iter (fun i ->
      match Xml.tag i with
          "periodic" ->
            if not (is_status_lock i) then begin
              if (not !setting_exist) then begin
                fprintf out_set "   <dl_settings name=\"Modules\">\n";
                setting_exist := true;
              end;
              fprintf out_set "      <dl_setting min=\"2\" max=\"3\" step=\"1\" var=\"%s\" shortname=\"%s\" values=\"START|STOP\"/>\n"
                (get_status_name i module_name) (get_status_shortname i)
            end
        | _ -> ())
      (Xml.children m))
    modules;
  if !setting_exist then fprintf out_set "   </dl_settings>\n";
  fprintf out_set " </dl_settings>\n";
  fprintf out_set "</settings>\n"

let h_name = "MODULES_H"

let () =
  if Array.length Sys.argv <> 5 then
    failwith (Printf.sprintf "Usage: %s out_settings_file default_freq fp_file xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(4)
  and fp_file = Sys.argv.(3)
  and default_freq = int_of_string(Sys.argv.(2))
  and out_set = open_out Sys.argv.(1) in
  try
    let xml = start_and_begin xml_file h_name in
    fprintf out_h "#define MODULES_IDLE  0\n";
    fprintf out_h "#define MODULES_RUN   1\n";
    fprintf out_h "#define MODULES_START 2\n";
    fprintf out_h "#define MODULES_STOP  3\n";
    nl ();
    (* Extract main_freq parameter *)
    let modules = try (ExtXml.child xml "modules") with _ -> Xml.Element("modules",[],[]) in
    let main_freq = try (int_of_string (Xml.attrib modules "main_freq")) with _ -> default_freq in
    freq := main_freq;
    fprintf out_h "#define MODULES_FREQUENCY %d\n" !freq;
    nl ();
    fprintf out_h "#ifdef MODULES_C\n";
    fprintf out_h "#define EXTERN_MODULES\n";
    fprintf out_h "#else\n";
    fprintf out_h "#define EXTERN_MODULES extern\n";
    fprintf out_h "#endif";
    nl ();
    (* Extract modules list *)
    let modules =
      try
        let target = Sys.getenv "TARGET" in
        GC.get_modules_of_config ~target xml (ExtXml.parse_file fp_file)
      with
      | Not_found -> failwith "TARTGET env needs to be specified to generate modules files"
    in
    (* Extract modules names (file name and module name) *)
    let modules_name =
      (List.map (fun m -> try Xml.attrib m.GC.xml "name" with _ -> "") modules) @
      (List.map (fun m -> m.GC.filename) modules) in
    (* Extract xml modules nodes *)
    let modules_list = List.map (fun m -> m.GC.xml) modules in
    check_dependencies modules_list modules_name;
    parse_modules modules_list;
    finish h_name;
    write_settings xml_file out_set modules_list;
    close_out out_set;
  with
      Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
    | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
    | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
    | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
