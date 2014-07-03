(*
 * Copyright (C) 2010 Gautier Hattenberger
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

(*
 * XML preprocessing for core autopilot
 *)
open Printf
open Xml2h

let (//) = Filename.concat

let paparazzi_conf = Env.paparazzi_home // "conf"
let autopilot_dir = paparazzi_conf // "autopilot"

(** Formatting with a margin *)
let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f ->
  fprintf out "%s" (String.make !margin ' ');
  fprintf out f

let get_ap_modes = fun ap ->
  List.filter (fun m -> (Xml.tag m) = "mode") (Xml.children ap)

let get_ap_control_block = fun ap ->
  List.filter (fun m -> (Xml.tag m) = "control_block") (Xml.children ap)

let get_ap_exceptions = fun ap ->
  try ExtXml.child ap "exceptions"
  with _ -> Xml.Element ("exceptions", [], [])

let get_mode_exceptions = fun mode ->
  List.filter (fun m -> (Xml.tag m) = "exception") (Xml.children mode)

let print_mode_name = fun name ->
  String.concat "" ["AUTOPILOT_MODE_"; (String.uppercase name)]

(** Define modes *)
let print_modes = fun modes out_h ->
  let mode_idx = ref 0 in
  List.iter (fun m ->
    try
      let name = Xml.attrib m "name" in
      lprintf out_h "#define %s %d\n" (print_mode_name name) !mode_idx;
      mode_idx := !mode_idx + 1;
    with _ -> ()
  )
    modes

(** Init function: set last_mode to initial ap_mode *) (* TODO really needed ? *)
let print_ap_init = fun modes out_h ->

  let find_default_mode = fun _ ->
    let default = List.find_all (fun m ->
      List.exists (fun s -> if Xml.tag s = "select" then Xml.attrib s "cond" = "$DEFAULT_MODE" else false) (Xml.children m)
    ) modes in
    match List.length default with
        0 -> List.hd modes
      | 1 -> List.hd default
      | _ -> failwith "Autopilot Core Error: only one default mode can be set"
  in

  let default = find_default_mode () in
  lprintf out_h "\nstatic inline void autopilot_core_init(void) {\n";
  right ();
  lprintf out_h "autopilot_mode = %s;\n" (print_mode_name (Xml.attrib default "name"));
  lprintf out_h "private_autopilot_mode = autopilot_mode;\n";
  lprintf out_h "last_autopilot_mode = autopilot_mode;\n";
  left ();
  lprintf out_h "}\n"

(** Function to test if a mode is selected *)
let print_test_select = fun modes out_h ->
  lprintf out_h "\nstatic inline uint8_t autopilot_core_mode_select(void) {\n";
  right ();
  List.iter (fun m -> (* Test select for all modes *)
    let select = List.filter (fun s -> Xml.tag s = "select") (Xml.children m) in
    List.iter (fun s -> (* In each mode, build condition and exceptions' list *)
      let cond = Xml.attrib s "cond" in
      let except = try String.concat " || "
                         (List.map (fun e -> Printf.sprintf "private_autopilot_mode != %s" (print_mode_name e))
                            (Str.split (Str.regexp "|") (Xml.attrib s "exception"))
                         ) with _ -> "" in
      match (cond, String.length except) with
          ("$DEFAULT_MODE", _) -> ()
        | (_, 0) -> lprintf out_h "if (%s) { return %s; }\n" cond (print_mode_name (Xml.attrib m "name"))
        | (_, _) -> lprintf out_h "if ((%s) && (%s)) { return %s; }\n" cond except (print_mode_name (Xml.attrib m "name"))
    ) select;
  ) modes;
  lprintf out_h "return private_autopilot_mode;\n";
  left ();
  lprintf out_h "}\n"


(** Function to test exceptions on modes
    *  The generated function returns the new mode if an exception is true
*)
let print_test_exception = fun modes out_h ->
  (** Test condition and deroute to given mode or last mode *)
  let print_exception = fun ex ->
    let name = Xml.attrib ex "deroute"
    and cond = Xml.attrib ex "cond" in
    match name with
        "$LAST_MODE" -> lprintf out_h "if (%s) { return last_autopilot_mode; }\n" cond
      | _ -> lprintf out_h "if (%s) { return %s; }\n" cond (print_mode_name name)
  in

  lprintf out_h "\nstatic inline uint8_t autopilot_core_mode_exceptions(uint8_t mode) {\n";
  right ();
  lprintf out_h "switch ( mode ) {\n";
  right ();
  List.iter (fun m -> (* Test exceptions for all modes *)
    lprintf out_h "case %s :\n" (print_mode_name (Xml.attrib m "name"));
    right ();
    lprintf out_h "{\n";
    right ();
    let exceptions = get_mode_exceptions m in
    List.iter (fun e ->
      print_exception e
    ) exceptions;
    left ();
    lprintf out_h "}\n";
    lprintf out_h "break;\n";
    left ()
  ) modes;
  left ();
  lprintf out_h "}\n";
  lprintf out_h "return mode;\n";
  left ();
  lprintf out_h "}\n"

(** Function to test global exceptions
    *  The generated function returns the mode of the last true exception in the list
*)
let print_global_exceptions = fun exceptions out_h ->
  lprintf out_h "\nstatic inline uint8_t autopilot_core_global_exceptions(uint8_t mode) {\n";
  right ();
  List.iter (fun ex -> lprintf out_h "if (%s) { mode = %s; }\n" (Xml.attrib ex "cond") (print_mode_name (Xml.attrib ex "deroute")))
    (Xml.children exceptions);
  lprintf out_h "return mode;\n";
  left ();
  lprintf out_h "}\n"

(** Set mode by calling start and stop functions if needed *)
let print_set_mode = fun modes out_h ->

  let print_case = fun mode f ->
    lprintf out_h "case %s :\n" (print_mode_name (Xml.attrib mode "name"));
    right ();
    List.iter (fun x -> lprintf out_h "%s;\n" x) (Str.split (Str.regexp "|") f);
    lprintf out_h "break;\n";
    left ();
  in
  let print_switch = fun var modes t ->
    lprintf out_h "// switch over %s functions\n" t;
    lprintf out_h "switch ( %s ) {\n" var;
    right ();
    List.iter (fun m ->
      try
        print_case m (Xml.attrib m t)
      with _ -> ()
    ) modes;
    left ();
    lprintf out_h "}\n"
  in

  lprintf out_h "\nstatic inline void autopilot_core_set_mode(uint8_t new_mode) {\n\n";
  right ();
  lprintf out_h "if (new_mode == private_autopilot_mode) return;\n\n"; (* set mode if different from current mode *)
  (* Print stop functions for each modes *)
  print_switch "private_autopilot_mode" modes "stop";
  fprintf out_h "\n";
  (* Print start functions for each modes *)
  print_switch "new_mode" modes "start";
  fprintf out_h "\n";
  lprintf out_h "last_autopilot_mode = private_autopilot_mode;\n";
  lprintf out_h "private_autopilot_mode = new_mode;\n";
  lprintf out_h "autopilot_mode = new_mode;\n";
  left ();
  lprintf out_h "}\n"


(** Peridiodic function: calls control loops according to the ap_mode *)
let print_ap_periodic = fun modes ctrl_block main_freq out_h ->

  (** Print function *)
  let print_call = fun call ->
    try
      let f = Xml.attrib call "fun" in
      let cond = try String.concat "" ["if ("; (Xml.attrib call "cond"); ") { "; f; "; }\n"]
        with _ -> String.concat "" [f; ";\n"] in
      lprintf out_h "%s" cond
    with _ -> ()
  in
  (** check control_block *)
  let get_control_block = fun c ->
    try
       List.find (fun n -> (Xml.attrib c "name") = (Xml.attrib n "name")) ctrl_block
     with
         Not_found -> failwith (sprintf "Could not find block %s" (Xml.attrib c "name"))
  in
  (** Print a control block *)
  let print_ctrl = fun ctrl ->
    List.iter (fun c ->
      match (Xml.tag c) with
          "call" -> print_call c
        | "call_block" -> List.iter print_call (Xml.children (get_control_block c))
        | _ -> ()
    ) (Xml.children ctrl)
  in
  (** Equivalent to the RunOnceEvery macro *)
  let print_prescaler = fun pre ctrl ->
    lprintf out_h "{\n";
    right ();
    lprintf out_h "static uint16_t prescaler = 0;\n";
    lprintf out_h "prescaler++;\n";
    lprintf out_h "if (prescaler >= %d) {\n" pre;
    right ();
    lprintf out_h "prescaler = 0;\n";
    print_ctrl ctrl;
    left ();
    lprintf out_h "}\n";
    left ();
    lprintf out_h "}\n"
  in
  let get_control = fun mode ->
    List.filter (fun m -> (Xml.tag m) = "control") (Xml.children mode)
  in

  (** Start printing the main periodic task *)
  lprintf out_h "\nstatic inline void autopilot_core_periodic_task(void) {\n\n";
  right ();
  lprintf out_h "uint8_t mode = autopilot_core_mode_select();\n"; (* get selected mode *)
  lprintf out_h "mode = autopilot_core_mode_exceptions(mode);\n"; (* change mode according to exceptions *)
  lprintf out_h "mode = autopilot_core_global_exceptions(mode);\n"; (* change mode according to global exceptions *)
  lprintf out_h "autopilot_core_set_mode(mode);\n\n"; (* set new mode and call start/stop functions *)
  lprintf out_h "switch ( private_autopilot_mode ) {\n";
  right ();
  List.iter (fun m -> (* Print control loops for each modes *)
    lprintf out_h "case %s :\n" (print_mode_name (Xml.attrib m "name"));
    right ();
    lprintf out_h "{\n";
    right ();
    List.iter (fun c -> (* Look for control loops *)
      let ctrl_freq = try int_of_string (Xml.attrib c "freq") with _ -> main_freq in
      let prescaler = main_freq / ctrl_freq in
      match prescaler with
          0 -> failwith (sprintf "Autopilot Core Error: control freq (%d) higher than main_freq (%d)" ctrl_freq main_freq)
        | 1 -> print_ctrl c (* no prescaler if running at main_freq *)
        | _ -> print_prescaler prescaler c
    ) (get_control m);
    left ();
    lprintf out_h "}\n";
    lprintf out_h "break;\n\n";
    left ();
  ) modes;
  left ();
  lprintf out_h "}\n";
  left ();
  lprintf out_h "}\n"

(** Parse config file *)
let parse_modes ap freq out_h =
  let modes = get_ap_modes ap in
  let ctrl_block = get_ap_control_block ap in
  print_modes modes out_h;
  fprintf out_h "\nEXTERN_AP uint8_t autopilot_mode;\n";
  fprintf out_h "\n#ifdef AUTOPILOT_CORE_C\n";
  fprintf out_h "\n#include \"modules.h\"\n";
  fprintf out_h "uint8_t private_autopilot_mode;\n";
  fprintf out_h "uint8_t last_autopilot_mode;\n\n";
  print_ap_init modes out_h;
  print_test_select modes out_h;
  print_test_exception modes out_h;
  print_global_exceptions (get_ap_exceptions ap) out_h;
  print_set_mode modes out_h;
  print_ap_periodic modes ctrl_block freq out_h;
  fprintf out_h "\n#endif // AUTOPILOT_CORE_C\n"

let h_name = "AUTOPILOT_CORE_H"

(** Main generation function
    * Usage: main_freq xml_file_input h_file_output
*)
let gen_autopilot main_freq xml_file h_file =
  let out_h = open_out h_file in
  try
    let ap_xml = start_and_begin_out xml_file h_name out_h in
    let _ = try
              let ap_name = Xml.attrib ap_xml "name" in
              fprintf out_h "/*** %s ***/\n\n" ap_name;
      with _ -> () in
    fprintf out_h "#ifdef AUTOPILOT_CORE_C\n";
    fprintf out_h "#define EXTERN_AP\n";
    fprintf out_h "#else\n";
    fprintf out_h "#define EXTERN_AP extern\n";
    fprintf out_h "#endif\n";
    parse_modes ap_xml main_freq out_h;
    fprintf out_h "\n#endif // %s\n" h_name;
    close_out out_h
  with
      Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
    | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
    | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
    | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1

(* Main call *)
let () =
  if Array.length Sys.argv <> 3 then
    failwith (Printf.sprintf "Usage: %s airframe_xml_file out_h_file" Sys.argv.(0));
  let xml_file = Sys.argv.(1)
  and h_file = Sys.argv.(2) in
  let (autopilot, ap_freq) = try
                               Gen_common.get_autopilot_of_airframe (Xml.parse_file xml_file)
    with
      Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
    | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
    | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
    | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
    | Not_found ->
      let out_h = open_out h_file in
      fprintf out_h "/*** Sorry, no autopilot file found ***/\n";
      close_out out_h;
      exit 0
  in
  try
    gen_autopilot ap_freq autopilot h_file;
    ()
  with
    Not_found -> fprintf stderr "gen_autopilot: What the heck? Something went wrong...\n"; exit 1
