(*
 * XML preprocessing for modules
 *
 * Copyright (C) 2009-2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

(** Formatting with a margin *)
let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f ->
  fprintf out "%s" (String.make !margin ' ');
  fprintf out f

let print_headers = fun out modules ->
  lprintf out  "#include \"std.h\"\n";
  List.iter (fun m ->
    let dirname = match m.Module.dir with None -> m.Module.name | Some d -> d in
    List.iter (fun h ->
      let dir = match h.Module.directory with None -> dirname | Some d -> d in
      Printf.fprintf out "#include \"%s/%s\"\n" dir h.Module.filename
    ) m.Module.headers
  ) modules

let get_status_name = fun f n ->
  let func = (Xml.attrib f "fun") in
  n^"_"^String.sub func 0 (try String.index func '(' with _ -> (String.length func))^"_status"

let get_status_shortname = fun f ->
  let func = (Xml.attrib f "fun") in
  String.sub func 0 (try String.index func '(' with _ -> (String.length func))

(*let fprint_status = fun ch mod_name p ->
  match p.autorun with
  | True | False ->
    Printf.fprintf ch "EXTERN_MODULES uint8_t %s;\n" (status_name mod_name p)
  | Lock -> ()

let fprint_periodic_init = fun ch mod_name p ->
  match p.autorun with
  | True -> Printf.fprintf ch "%s = %s;" (status_name mod_name p) "MODULES_START"
  | False -> Printf.fprintf ch "%s = %s;" (status_name mod_name p) "MODULES_IDLE"
  | Lock -> ()

let fprint_init = fun ch init -> Printf.fprintf ch "%s;\n" init
*)

let get_period_and_freq = fun f ->
  let period = try Some (Xml.attrib f "period") with _ -> None
  and freq = try Some (Xml.attrib f "freq") with _ -> None in
  match period, freq with
    | None, None -> ("(1.f / MODULES_FREQUENCY)", "(MODULES_FREQUENCY)")
    | Some _p, None -> ("("^_p^")", "(1. / ("^_p^"))")
    | None, Some _f -> ("(1. / ("^_f^"))", "("^_f^")")
    | Some _p, Some _ ->
      fprintf stderr "Warning: both period and freq are defined but only period is used for function %s\n" (ExtXml.attrib f "fun");
      ("("^_p^")", "(1. / ("^_p^"))")

(*let fprint_period_freq = fun ch max_freq p ->
  let period, freq = match p.period_freq with
    | Unset -> 1. /. max_freq, max_freq
    | Set (p, f) -> p, f in
  let cap_fname = Compat.uppercase_ascii p.fname in
  Printf.fprintf ch "#define %s_PERIOD %f\n" cap_fname period;
  Printf.fprintf ch "#define %s_FREQ %f\n" cap_fname freq*)

(* Extract function name and return in capital letters *)
let get_cap_name = fun f ->
  let name = Str.full_split (Str.regexp "[()]") f in
  match name with
    | [Str.Text t]
    | [Str.Text t; Str.Delim "("; Str.Delim ")"]
    | [Str.Text t; Str.Delim "("; Str.Text _ ; Str.Delim ")"] -> Compat.uppercase_ascii t
    | _ -> failwith "Gen_modules: not a valid function name"


(** Computes the required modulos *)
let get_functions_modulos = fun modules ->
  let found_modulos = Hashtbl.create 10 in
  let idx = ref 0 in
  let delay = ref 0. in (** Basic balancing: increment with a step of 0.1 *)
  let functions_modulo = List.map (fun m ->
    let periodic = List.filter (fun i -> (String.compare (Xml.tag i) "periodic") == 0) (Xml.children m.Module.xml) in
    let module_name = m.Module.name in
    List.map (fun x ->
      let p, _ = get_period_and_freq x in
      let d = begin try
        let _d = float_of_string (Xml.attrib x "delay") in
        if _d > 0.99 then
          begin
            fprintf stderr "Warning: 'delay' attribute should be a float value between 0. and 1.\n";
            _d /. 65536.
          end
        else _d (* try to keep some backward compatibility *)
        with _ ->
          delay := !delay +. 0.1;
          if !delay > 0.9 then delay := 0.;
          !delay
        end
      in
      try
        ((x, module_name, d), (p, Hashtbl.find found_modulos p))
      with Not_found ->
        incr idx; (* create new modulo *)
        Hashtbl.add found_modulos p !idx;
        ((x, module_name, d), (p, !idx))
    ) periodic
  ) modules in
  List.flatten functions_modulo


let print_function_freq = fun out modules ->
  fprintf out "\n";
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
          "periodic" ->
            let fname = get_cap_name (Xml.attrib i "fun") in
            let p, f = get_period_and_freq i in
            lprintf out "#define %s_PERIOD %s\n" fname p;
            lprintf out "#define %s_FREQ %s\n" fname f;
        | _ -> ())
      (Xml.children m.Module.xml))
    modules

let print_function_prescalers = fun out functions_modulo ->
  fprintf out "\n";
  let found_modulos = Hashtbl.create 10 in
  List.iter (fun (_, (p, m)) ->
    if not (Hashtbl.mem found_modulos m) then
      lprintf out "#define PRESCALER_%d (uint32_t)(MODULES_FREQUENCY * %s)\n" m p
    else
      Hashtbl.add found_modulos m p
  ) functions_modulo


let is_status_lock = fun p ->
  let mode = ExtXml.attrib_or_default p "autorun" "LOCK" in
  mode = "LOCK"

let print_status = fun out modules ->
  fprintf out "\n";
  List.iter (fun m ->
    let module_name = m.Module.name in
    List.iter (fun i ->
      match Xml.tag i with
          "periodic" ->
            if not (is_status_lock i) then begin
              lprintf out "EXTERN_MODULES uint8_t %s;\n" (get_status_name i module_name);
            end
        | _ -> ())
      (Xml.children m.Module.xml))
    modules

let modules_of_task = fun modules ->
  let h = Hashtbl.create 1 in
  List.iter (fun m ->
    let task = match m.Module.task with None -> "default" | Some t -> t in
    if Hashtbl.mem h task then
      Hashtbl.replace h task (List.append (Hashtbl.find h task) [m])
    else
      Hashtbl.add h task [m]
  ) modules;
  h

let print_init = fun out task modules ->
  lprintf out "\nstatic inline void modules_%s_init(void) {\n" task;
  right ();
  List.iter (fun m ->
    let module_name = m.Module.name in
    List.iter (fun i ->
      match Xml.tag i with
          "init" -> lprintf out "%s;\n" (Xml.attrib i "fun")
        | "periodic" -> if not (is_status_lock i) then
            lprintf out "%s = %s;\n" (get_status_name i module_name) (try match Xml.attrib i "autorun" with
                "TRUE" | "true" -> "MODULES_START"
              | "FALSE" | "false" | "LOCK" | "lock" -> "MODULES_IDLE"
              | _ -> failwith "Error: Unknown autorun value (possible values are: TRUE, FALSE, LOCK(default))"
              with _ -> "MODULES_IDLE" (* this should not be possible anyway *))
        | _ -> ())
      (Xml.children m.Module.xml))
    modules;
  left ();
  lprintf out "}\n"

let print_init_functions = fun out modules ->
  let h = modules_of_task modules in
  Hashtbl.iter (print_init out) h;
  lprintf out "\nstatic inline void modules_init(void) {\n";
  right ();
  Hashtbl.iter (fun t _ -> lprintf out "modules_%s_init();\n" t) h;
  left ();
  lprintf out "}\n"


let print_periodic = fun out functions_modulo task modules ->
  (* filter for a given task *)
  let functions_modulo = List.filter (fun m ->
    let (_, name, _), _ = m in
    List.exists (fun m' -> m'.Module.name = name) modules
  ) functions_modulo in
  (* start printing *)
  lprintf out "\nstatic inline void modules_%s_periodic_task(void) {\n" task;
  right ();
  let modulos = GC.singletonize (List.map (fun (_, (_, m)) -> m) functions_modulo) in
  (** Print modulos *)
  List.iter (fun modulo ->
    let v = sprintf "i%d" modulo in
    lprintf out "static uint32_t %s; %s++; if (%s>=PRESCALER_%d) %s=0;\n" v v v modulo v;)
    modulos;
  (** Print start and stop functions *)
  List.iter (fun m ->
    let module_name = m.Module.name in
    let periodic = List.filter (fun i -> (String.compare (Xml.tag i) "periodic") == 0) (Xml.children m.Module.xml) in
    List.iter (fun f ->
      if (is_status_lock f) then begin
        try
          let start = (Xml.attrib f "start") in
          fprintf out "\n";
          lprintf out "%s;\n" start
        with _ -> ();
        try let stop = Xml.attrib f "stop" in fprintf stderr "Warning: stop %s function will not be called\n" stop with _ -> ();
      end
      else begin
        let status = get_status_name f module_name in
        fprintf out "\n";
        lprintf out "if (%s == MODULES_START) {\n" status;
        right ();
        ignore(try lprintf out "%s;\n" (Xml.attrib f "start") with _ -> ());
        lprintf out "%s = MODULES_RUN;\n" status;
        left ();
        lprintf out "}\n";
        lprintf out "if (%s == MODULES_STOP) {\n" status;
        right ();
        ignore(try lprintf out "%s;\n" (Xml.attrib f "stop") with _ -> ());
        lprintf out "%s = MODULES_IDLE;\n" status;
        left ();
        lprintf out "}\n";
      end
    ) periodic
  ) modules;
  (** Print periodic functions *)
  let functions = List.sort (fun (_,p) (_,p') -> compare p p') functions_modulo in
  fprintf out "\n";
  List.iter (fun ((func, name, delay), (p, m)) ->
    if (List.exists (fun _module -> _module.Module.name = name) modules) then begin
      let function_name = ExtXml.attrib func "fun" in
      let p, f = get_period_and_freq func in
      if f = "(MODULES_FREQUENCY)" then
        begin
          if (is_status_lock func) then
            lprintf out "%s;\n" function_name
          else begin
            lprintf out "if (%s == MODULES_RUN) {\n" (get_status_name func name);
            right ();
            lprintf out "%s;\n" function_name;
            left ();
            lprintf out "}\n";
          end
        end
      else
        begin
          let run = if not (is_status_lock func) then sprintf " && %s == MODULES_RUN" (get_status_name func name)
            else ""
          in
          lprintf out "if (i%d == (uint32_t)(%ff * PRESCALER_%d)%s) {\n" m delay m run;
          right ();
          lprintf out "%s;\n" function_name;
          left ();
          lprintf out "}\n"
        end;
    end
  ) functions;
  left ();
  lprintf out "}\n"

let print_periodic_functions = fun out functions_modulo modules ->
  let h = modules_of_task modules in
  Hashtbl.iter (print_periodic out functions_modulo) h;
  lprintf out "\nstatic inline void modules_periodic_task(void) {\n";
  right ();
  Hashtbl.iter (fun t _ -> lprintf out "modules_%s_periodic_task();\n" t) h;
  left ();
  lprintf out "}\n"


let print_event = fun out task modules ->
  lprintf out "\nstatic inline void modules_%s_event_task(void) {\n" task;
  right ();
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
          "event" -> lprintf out "%s;\n" (Xml.attrib i "fun")
        | _ -> ())
      (Xml.children m.Module.xml))
    modules;
  left ();
  lprintf out "}\n"

let print_event_functions = fun out modules ->
  let h = modules_of_task modules in
  Hashtbl.iter (print_event out) h;
  lprintf out "\nstatic inline void modules_event_task(void) {\n";
  right ();
  Hashtbl.iter (fun t _ -> lprintf out "modules_%s_event_task();\n" t) h;
  left ();
  lprintf out "}\n"


let print_datalink_functions = fun out modules ->
  lprintf out "\n#include \"pprzlink/messages.h\"\n";
  lprintf out "#include \"generated/airframe.h\"\n";
  lprintf out "static inline void modules_parse_datalink(uint8_t msg_id __attribute__ ((unused)),
                                          struct link_device *dev __attribute__((unused)),
                                          struct transport_tx *trans __attribute__((unused)),
                                          uint8_t *buf __attribute__((unused))) {\n";
  right ();
  let else_ = ref "" in
  List.iter (fun m ->
    List.iter (fun i ->
      match Xml.tag i with
          "datalink" ->
            lprintf out "%sif (msg_id == DL_%s) { %s; }\n" !else_ (ExtXml.attrib i "message") (ExtXml.attrib i "fun");
            else_ := "else "
        | _ -> ())
      (Xml.children m.Module.xml))
    modules;
  left ();
  lprintf out "}\n"

let parse_modules out modules =
  print_headers out modules;
  print_function_freq out modules;
  let functions_modulo = get_functions_modulos modules in
  print_function_prescalers out functions_modulo;
  print_status out modules;
  fprintf out "\n";
  print_init_functions out modules;
  print_periodic_functions out functions_modulo modules;
  print_event_functions out modules;
  fprintf out "\n";
  fprintf out "#ifdef MODULES_DATALINK_C\n";
  print_datalink_functions out modules;
  fprintf out "\n";
  fprintf out "#endif // MODULES_DATALINK_C\n"

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
      let module_name = m.Module.name in
      let dep_string = get_pcdata m.Module.xml "depends" in
      (*fprintf stderr "\n\nWARNING: parsing dep string: %s\n\n" dep_string;
      fprintf stderr "\n\nWARNING: names: %s" (String.concat "," names);*)
      let require = deps_of_string dep_string in
      List.iter (fun deps ->
        (* iterate over all dependencies, where the second level contains the OR dependencies *)
        let find_common satisfied d = if List.mem d names then d::satisfied else satisfied in
        let satisfied = List.fold_left find_common [] deps in
        if List.length satisfied == 0 then
          begin
            fprintf stderr "\nDEPENDENCY Error: Module %s requires %s\n" module_name (String.concat " or " deps);
            fprintf stderr "Available loaded modules are:\n    %s\n\n" (String.concat "\n    " names);
            exit 1
          end)
        require;
      let conflict_string = get_pcdata m.Module.xml "conflicts" in
      let conflict_l = List.flatten (deps_of_string conflict_string) in
      List.iter (fun con ->
        if List.exists (fun c -> String.compare c con == 0) names then
          fprintf stderr "\nDEPENDENCY WARNING: Module %s conflicts with %s\n" module_name con)
        conflict_l
    with _ -> ()
  ) modules


let h_name = "MODULES_H"

let generate = fun modules xml_file out_file ->
  let out = open_out out_file in

  begin_out out xml_file h_name;
  define_out out "MODULES_IDLE " "0";
  define_out out "MODULES_RUN  " "1";
  define_out out "MODULES_START" "2";
  define_out out "MODULES_STOP " "3";
  fprintf out "\n";

  fprintf out "#ifndef MODULES_FREQUENCY\n";
  fprintf out "#ifdef PERIODIC_FREQUENCY\n";
  fprintf out "#define MODULES_FREQUENCY PERIODIC_FREQUENCY\n";
  fprintf out "#else\n";
  fprintf out "#error \"neither MODULES_FREQUENCY or PERIODIC_FREQUENCY are defined\"\n";
  fprintf out "#endif\n";
  fprintf out "#endif\n";
  fprintf out "\n";
  fprintf out "#ifdef MODULES_C\n";
  fprintf out "#define EXTERN_MODULES\n";
  fprintf out "#else\n";
  fprintf out "#define EXTERN_MODULES extern\n";
  fprintf out "#endif\n";

  parse_modules out modules;

  finish_out out h_name;
  close_out out

