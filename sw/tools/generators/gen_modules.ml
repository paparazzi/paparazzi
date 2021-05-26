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
module GC = Gen_common

(** TODO make this a config file ?? *)
let tasks_order = ["mcu"; "core"; "sensors"; "estimation"; "radio_control"; "control"; "actuators"; "datalink"; "default"]

(** Formatting with a margin *)
let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f ->
  fprintf out "%s" (String.make !margin ' ');
  fprintf out f

(* Encapsulate a condition attribute for function calls *)
let lprintf_with_cond out s = function
  | None -> lprintf out "%s;\n" s
  | Some cond ->
      let cond_expr = Fp_proc.parse_expression cond in
      fprintf out "#if %s\n%s%s;\n#endif\n" (Expr_syntax.sprint cond_expr) (String.make !margin ' ') s

let print_headers = fun out modules ->
  lprintf out  "#include \"std.h\"\n";
  List.iter (fun m ->
    let dirname = match m.Module.dir with None -> m.Module.name | Some d -> d in
    List.iter (fun h ->
      let dir = match h.Module.directory with None -> dirname | Some d -> d in
      let inc = sprintf "#include \"%s/%s\"" dir h.Module.filename in
      match h.Module.filecond with
      | None -> fprintf out "%s\n" inc
      | Some cond -> fprintf out "#if %s\n%s\n#endif\n" cond inc
    ) m.Module.headers
  ) modules

let get_status_name = fun f n ->
  n^"_"^String.sub f 0 (try String.index f '(' with _ -> (String.length f))^"_status"

let get_period_and_freq = function
  | Module.Unset -> ("(1.f / MODULES_FREQUENCY)", "(MODULES_FREQUENCY)")
  | Module.Freq _f -> ("(1.f / ("^_f^"))", "("^_f^")")
  | Module.Period _p -> ("("^_p^")", "(1.f / ("^_p^"))")
  | Module.Set (_p, _f) -> (Printf.sprintf "(%ff)" _p, Printf.sprintf "(%ff)" _f)

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
    let module_name = m.Module.name in
    List.map (fun x ->
      let p, _ = get_period_and_freq x.Module.period_freq in
      let d = match x.Module.delay with
        | Some _d ->
            if _d > 0.99 then
              begin
                fprintf stderr "Warning: 'delay' attribute should be a float value between 0. and 1.\n";
                _d /. 65536.
              end
            else _d (* try to keep some backward compatibility *)
        | None ->
            delay := !delay +. 0.1;
            if !delay > 0.9 then delay := 0.;
            !delay
      in
      try
        ((x, module_name, d), (p, Hashtbl.find found_modulos p))
      with Not_found ->
        incr idx; (* create new modulo *)
        Hashtbl.add found_modulos p !idx;
        ((x, module_name, d), (p, !idx))
    ) m.Module.periodics
  ) modules in
  List.flatten functions_modulo


let print_function_freq = fun out modules ->
  fprintf out "\n";
  List.iter (fun m ->
    List.iter (fun i ->
      let fname = Compat.uppercase_ascii i.Module.fname in
      let p, f = get_period_and_freq i.Module.period_freq in
      lprintf out "#define %s_PERIOD %s\n" fname p;
      lprintf out "#define %s_FREQ %s\n" fname f;
    ) m.Module.periodics
  ) modules

let print_function_prescalers = fun out functions_modulo ->
  fprintf out "\n";
  let found_modulos = Hashtbl.create 10 in
  List.iter (fun (_, (p, m)) ->
    if not (Hashtbl.mem found_modulos m) then begin
      lprintf out "#define PRESCALER_%d (uint32_t)(MODULES_FREQUENCY * %s)\n" m p;
      Hashtbl.add found_modulos m p
    end
  ) functions_modulo


let is_status_lock = fun p ->
  match p.Module.autorun with Module.Lock -> true | _ -> false

let print_status = fun out modules ->
  fprintf out "\n";
  List.iter (fun m ->
    let module_name = m.Module.name in
    List.iter (fun p ->
      if not (is_status_lock p) then
        lprintf out "EXTERN_MODULES uint8_t %s;\n" (get_status_name p.Module.fname module_name)
    ) m.Module.periodics
  ) modules

(** Create an order list of pairs (task name, list of modules in task)
  *
  * FIXME for now it goes with a hashtbl first then a list with hard-coded order
  * and finally add (and display warning) all extra tasks group
  * This could be made more configurable
  *)
let modules_of_task = fun modules ->
  let h = Hashtbl.create 1 in
  List.iter (fun m ->
    let task = match m.Module.task with None -> "default" | Some t -> t in
    if Hashtbl.mem h task then
      Hashtbl.replace h task (List.append (Hashtbl.find h task) [m])
    else
      Hashtbl.add h task [m]
  ) modules;
  let mot = List.map (fun task_name ->
    let task_list = try Hashtbl.find h task_name with Not_found -> [] in
    Hashtbl.remove h task_name;
    (task_name, task_list)
  ) tasks_order in
  let mot = Hashtbl.fold (fun key tasks l ->
    Printf.eprintf "Warning: adding an unknown task '%s'\n" key;
    l @ [(key, tasks)]
  ) h mot in
  mot

let print_init = fun out (task, modules) ->
  lprintf out "\nstatic inline void modules_%s_init(void) {\n" task;
  right ();
  List.iter (fun m ->
    let module_name = m.Module.name in
    List.iter (fun init ->
      lprintf_with_cond out init.Module.iname init.Module.cond
    ) m.Module.inits;
    List.iter (fun p ->
      match p.Module.autorun with
      | Module.True -> lprintf out "%s = MODULES_START;\n" (get_status_name p.Module.fname  module_name)
      | Module.False -> lprintf out "%s = MODULES_STOP;\n" (get_status_name p.Module.fname  module_name)
      | _ -> ()
    ) m.Module.periodics
  ) modules;
  left ();
  lprintf out "}\n"

let print_init_functions = fun out modules ->
  let mot = modules_of_task modules in
  List.iter (print_init out) mot;
  lprintf out "\nstatic inline void modules_init(void) {\n";
  right ();
  List.iter (fun (t, _) -> lprintf out "modules_%s_init();\n" t) mot;
  left ();
  lprintf out "}\n"


let print_periodic = fun out functions_modulo (task, modules) ->
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
  let lprint_opt = fun f cond ->
    match f with Some s -> lprintf_with_cond out s cond | None -> ()
  in
  List.iter (fun m ->
    let module_name = m.Module.name in
    List.iter (fun p ->
      match p.Module.autorun with
      | Module.Lock ->
          lprint_opt p.Module.start p.Module.cond;
          begin match p.Module.stop with
          | Some stop -> fprintf stderr "Warning: stop %s function will not be called\n" stop
          | _ -> () end
      | _ -> (* not locked *)
          let status = get_status_name p.Module.fname module_name in
          fprintf out "\n";
          lprintf out "if (%s == MODULES_START) {\n" status;
          right ();
          lprint_opt p.Module.start p.Module.cond; 
          lprintf out "%s = MODULES_RUN;\n" status;
          left ();
          lprintf out "}\n";
          lprintf out "if (%s == MODULES_STOP) {\n" status;
          right ();
          lprint_opt p.Module.stop p.Module.cond; 
          lprintf out "%s = MODULES_IDLE;\n" status;
          left ();
          lprintf out "}\n"
    ) m.Module.periodics
  ) modules;
  (** Print periodic functions *)
  let functions = List.sort (fun (_,p) (_,p') -> compare p p') functions_modulo in
  fprintf out "\n";
  List.iter (fun ((periodic, name, delay), (p, m)) ->
    if (List.exists (fun _module -> _module.Module.name = name) modules) then begin
      let p, f = get_period_and_freq periodic.Module.period_freq in
      if f = "(MODULES_FREQUENCY)" then
        begin
          match periodic.Module.autorun with
          | Module.Lock ->
              lprintf_with_cond out periodic.Module.call periodic.Module.cond
          | _ ->
              lprintf out "if (%s == MODULES_RUN) {\n" (get_status_name periodic.Module.fname name);
              right ();
              lprintf_with_cond out periodic.Module.call periodic.Module.cond;
              left ();
              lprintf out "}\n"
        end
      else
        begin
          let run = match periodic.Module.autorun with
            | Module.Lock -> ""
            | _ -> sprintf " && %s == MODULES_RUN" (get_status_name periodic.Module.fname name)
          in
          lprintf out "if (i%d == (uint32_t)(%ff * PRESCALER_%d)%s) {\n" m delay m run;
          right ();
          lprintf_with_cond out periodic.Module.call periodic.Module.cond;
          left ();
          lprintf out "}\n"
        end;
    end
  ) functions;
  left ();
  lprintf out "}\n"

let print_periodic_functions = fun out functions_modulo modules ->
  let mot = modules_of_task modules in
  List.iter (print_periodic out functions_modulo) mot;
  lprintf out "\nstatic inline void modules_periodic_task(void) {\n";
  right ();
  List.iter (fun (t, _) -> lprintf out "modules_%s_periodic_task();\n" t) mot;
  left ();
  lprintf out "}\n"


let print_event = fun out (task, modules) ->
  lprintf out "\nstatic inline void modules_%s_event_task(void) {\n" task;
  right ();
  List.iter (fun m ->
    List.iter (fun i ->
      lprintf_with_cond out i.Module.ev i.Module.cond
    ) m.Module.events
  ) modules;
  left ();
  lprintf out "}\n"

let print_event_functions = fun out modules ->
  let mot = modules_of_task modules in
  List.iter (print_event out) mot;
  lprintf out "\nstatic inline void modules_event_task(void) {\n";
  right ();
  List.iter (fun (t, _) -> lprintf out "modules_%s_event_task();\n" t) mot;
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
    List.iter (fun d ->
      lprintf out "%sif (msg_id == DL_%s) { %s; }\n" !else_ d.Module.message d.Module.func;
      else_ := "else "
    ) m.Module.datalinks
  ) modules;
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

let h_name = "MODULES_H"

let generate = fun modules xml_file out_file ->
  let out = open_out out_file in

  Xml2h.begin_out out xml_file h_name;
  Xml2h.define_out out "MODULES_IDLE " "0";
  Xml2h.define_out out "MODULES_RUN  " "1";
  Xml2h.define_out out "MODULES_START" "2";
  Xml2h.define_out out "MODULES_STOP " "3";
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

  Xml2h.finish_out out h_name;
  close_out out

