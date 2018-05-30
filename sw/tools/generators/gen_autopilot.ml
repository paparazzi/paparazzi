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

let get_state_machines = fun ap ->
  List.filter (fun m -> (Xml.tag m) = "state_machine") (Xml.children ap)

let get_modes = fun sm ->
  List.filter (fun m -> (Xml.tag m) = "mode") (Xml.children sm)

let get_ap_control_block = fun ap ->
  List.filter (fun m -> (Xml.tag m) = "control_block") (Xml.children ap)

let get_exceptions = fun sm ->
  try ExtXml.child sm "exceptions"
  with _ -> Xml.Element ("exceptions", [], [])

let get_includes = fun sm ->
  try ExtXml.child sm "includes"
  with _ -> Xml.Element ("includes", [], [])

let get_settings = fun sm ->
  try ExtXml.child sm "settings"
  with _ -> Xml.Element ("settings", [], [])

let has_modules = fun sm ->
  try
    let m = ExtXml.child sm "modules" in
    List.length (Xml.children m) > 0
  with _ -> false

let get_mode_exceptions = fun mode ->
  List.filter (fun m -> (Xml.tag m) = "exception") (Xml.children mode)

let print_includes = fun includes out_h ->
  List.iter (fun i ->
    match Xml.tag i with
    | "include" ->
      let name = ExtXml.attrib i "name" in
      lprintf out_h "#include \"%s\"\n" name
    | "define" ->
      let name = ExtXml.attrib i "name"
      and value = ExtXml.attrib i "value"
      and cond = try Some (Xml.attrib i "cond") with _ -> None in
      begin match cond with
      | None -> lprintf out_h "#define %s %s\n" name value
      | Some c ->
          lprintf out_h "#%s\n" c;
          lprintf out_h "#define %s %s\n" name value;
          lprintf out_h "#endif\n"
      end
    | _ -> failwith "[gen_autopilot] Unknown includes type"
  ) (Xml.children includes)

let print_mode_name = fun sm_name name ->
  String.concat "" [(Compat.uppercase_ascii sm_name); "_MODE_"; (Compat.uppercase_ascii name)]

(** Define modes *)
let print_modes = fun modes sm_name out_h ->
  let mode_idx = ref 0 in
  List.iter (fun m ->
    try
      let name = Xml.attrib m "name" in
      lprintf out_h "#define %s %d\n" (print_mode_name sm_name name) !mode_idx;
      mode_idx := !mode_idx + 1;
    with _ -> ()
  ) modes

(** Init function: set last_mode to initial ap_mode *) (* TODO really needed ? *)
let print_ap_init = fun modes name out_h ->

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
  lprintf out_h "\nstatic inline void autopilot_core_%s_init(void) {\n" name;
  right ();
  lprintf out_h "autopilot_mode_%s = %s;\n" name (print_mode_name name (Xml.attrib default "name"));
  lprintf out_h "private_autopilot_mode_%s = autopilot_mode_%s;\n" name name;
  lprintf out_h "last_autopilot_mode_%s = autopilot_mode_%s;\n" name name;
  left ();
  lprintf out_h "}\n"

(** Function to test if a mode is selected *)
let print_test_select = fun modes name out_h ->
  lprintf out_h "\nstatic inline uint8_t autopilot_core_%s_mode_select(void) {\n" name;
  right ();
  List.iter (fun m -> (* Test select for all modes *)
    let select = List.filter (fun s -> Xml.tag s = "select") (Xml.children m) in
    List.iter (fun s -> (* In each mode, build condition and exceptions' list *)
      let cond = Xml.attrib s "cond" in
      let except = try String.concat " || "
                         (List.map (fun e -> Printf.sprintf "private_autopilot_mode_%s != %s" name (print_mode_name name e))
                            (Str.split (Str.regexp "|") (Xml.attrib s "exception"))
                         ) with _ -> "" in
      match (cond, String.length except) with
          ("$DEFAULT_MODE", _) -> ()
        | (_, 0) -> lprintf out_h "if (%s) { return %s; }\n" cond (print_mode_name name (Xml.attrib m "name"))
        | (_, _) -> lprintf out_h "if ((%s) && (%s)) { return %s; }\n" cond except (print_mode_name name (Xml.attrib m "name"))
    ) select;
  ) modes;
  lprintf out_h "return private_autopilot_mode_%s;\n" name;
  left ();
  lprintf out_h "}\n"


(** Function to test exceptions on modes
 *  The generated function returns the new mode if an exception is true
*)
let print_test_exception = fun modes name out_h ->
  (** Test condition and deroute to given mode or last mode *)
  let print_exception = fun ex ->
    let deroute = Xml.attrib ex "deroute"
    and cond = Xml.attrib ex "cond" in
    match deroute with
      | "$LAST_MODE" -> lprintf out_h "if (%s) { return last_autopilot_mode_%s; }\n" cond name
      | _ -> lprintf out_h "if (%s) { return %s; }\n" cond (print_mode_name name deroute)
  in

  lprintf out_h "\nstatic inline uint8_t autopilot_core_%s_mode_exceptions(uint8_t mode) {\n" name;
  right ();
  lprintf out_h "switch ( mode ) {\n";
  right ();
  List.iter (fun m -> (* Test exceptions for all modes *)
    lprintf out_h "case %s :\n" (print_mode_name name (Xml.attrib m "name"));
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
  lprintf out_h "default:\n";
  right ();
  lprintf out_h "break;\n";
  left ();
  left ();
  lprintf out_h "}\n";
  lprintf out_h "return mode;\n";
  left ();
  lprintf out_h "}\n"

(** Function to test global exceptions
    *  The generated function returns the mode of the last true exception in the list
*)
let print_global_exceptions = fun exceptions name out_h ->
  lprintf out_h "\nstatic inline uint8_t autopilot_core_%s_global_exceptions(uint8_t mode) {\n" name;
  right ();
  List.iter (fun ex -> lprintf out_h "if (%s) { mode = %s; }\n" (Xml.attrib ex "cond") (print_mode_name name (Xml.attrib ex "deroute")))
    (Xml.children exceptions);
  lprintf out_h "return mode;\n";
  left ();
  lprintf out_h "}\n"

(** Set mode by calling start and stop functions if needed *)
let print_set_mode = fun modes name out_h ->

  let print_case = fun mode f ->
    lprintf out_h "case %s :\n" (print_mode_name name (Xml.attrib mode "name"));
    right ();
    List.iter (fun x -> lprintf out_h "%s;\n" (ExtXml.attrib x "fun")) (Xml.children f);
    lprintf out_h "break;\n";
    left ();
  in
  let print_switch = fun var modes t ->
    lprintf out_h "// switch over %s functions\n" t;
    lprintf out_h "switch ( %s ) {\n" var;
    right ();
    List.iter (fun m ->
      try
        print_case m (ExtXml.child m t)
      with _ -> ()
    ) modes;
    lprintf out_h "default:\n";
    right ();
    lprintf out_h "break;\n";
    left ();
    left ();
    lprintf out_h "}\n"
  in

  lprintf out_h "\nstatic inline void autopilot_core_%s_set_mode(uint8_t new_mode) {\n\n" name;
  right ();
  lprintf out_h "if (new_mode == private_autopilot_mode_%s) return;\n\n" name; (* set mode if different from current mode *)
  (* Print stop functions for each modes *)
  print_switch ("private_autopilot_mode_"^name) modes "on_exit";
  fprintf out_h "\n";
  (* Print start functions for each modes *)
  print_switch "new_mode" modes "on_enter";
  fprintf out_h "\n";
  lprintf out_h "last_autopilot_mode_%s = private_autopilot_mode_%s;\n" name name;
  lprintf out_h "private_autopilot_mode_%s = new_mode;\n" name;
  lprintf out_h "autopilot_mode_%s = new_mode;\n" name;
  left ();
  lprintf out_h "}\n"


(** Peridiodic function: calls control loops according to the ap_mode *)
let print_ap_periodic = fun modes ctrl_block main_freq name out_h ->

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
  let print_prescaler = fun freq ctrl ->
    lprintf out_h "{\n";
    right ();
    lprintf out_h "static uint16_t prescaler = 0;\n";
    lprintf out_h "prescaler++;\n";
    lprintf out_h "if (prescaler >= (uint16_t)(%s / %s)) {\n" main_freq freq;
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
  lprintf out_h "\nstatic inline void autopilot_core_%s_periodic_task(void) {\n\n" name;
  right ();
  lprintf out_h "uint8_t mode = autopilot_core_%s_mode_select();\n" name; (* get selected mode *)
  lprintf out_h "mode = autopilot_core_%s_mode_exceptions(mode);\n" name; (* change mode according to exceptions *)
  lprintf out_h "mode = autopilot_core_%s_global_exceptions(mode);\n" name; (* change mode according to global exceptions *)
  lprintf out_h "autopilot_core_%s_set_mode(mode);\n\n" name; (* set new mode and call start/stop functions *)
  lprintf out_h "switch ( private_autopilot_mode_%s ) {\n" name;
  right ();
  List.iter (fun m -> (* Print control loops for each modes *)
    lprintf out_h "case %s :\n" (print_mode_name name (Xml.attrib m "name"));
    right ();
    lprintf out_h "{\n";
    right ();
    List.iter (fun c -> (* Look for control loops *)
      let ctrl_freq = try Some (Xml.attrib c "freq") with _ -> None in
      (* TODO test if possible to determine freq and if valid
       * let prescaler = main_freq / ctrl_freq in
       *  0 -> failwith (sprintf "Autopilot Core Error: control freq (%d) higher than main_freq (%d)" ctrl_freq main_freq)
       *)
      match ctrl_freq with
        | None -> print_ctrl c (* no prescaler, runing at main freq *)
        | Some freq -> print_prescaler freq c
    ) (get_control m);
    left ();
    lprintf out_h "}\n";
    lprintf out_h "break;\n\n";
    left ();
  ) modes;
  lprintf out_h "default:\n";
  right ();
  lprintf out_h "break;\n";
  left ();
  left ();
  lprintf out_h "}\n";
  left ();
  lprintf out_h "}\n"

(** Parse config file and generate output header *)
let parse_and_gen_modes xml_file ap_name main_freq h_dir sm =
  (* Open temp file with correct permissions *)
  let tmp_file, out_h = Filename.open_temp_file "pprz_ap_" ".h" in
  Unix.chmod tmp_file 0o644;
  try
    (* Get state machine name *)
    let name = Xml.attrib sm "name" in
    let name_up = Compat.uppercase_ascii name in
    (* Generate start of header *)
    begin_out xml_file ("AUTOPILOT_CORE_"^name_up^"_H") out_h;
    fprintf out_h "/*** %s ***/\n\n" ap_name;
    (* Print EXTERN definition *)
    fprintf out_h "#ifdef AUTOPILOT_CORE_%s_C\n" name_up;
    fprintf out_h "#define EXTERN_%s\n" name_up;
    fprintf out_h "#else\n";
    fprintf out_h "#define EXTERN_%s extern\n" name_up;
    fprintf out_h "#endif\n";
    (* Get modes and control blocks *)
    let modes = get_modes sm in
    let ctrl_block = get_ap_control_block sm in
    (* Print mode names and variable *)
    print_modes modes name_up out_h;
    fprintf out_h "\nEXTERN_%s uint8_t autopilot_mode_%s;\n" name_up name;
    fprintf out_h "\n#ifdef AUTOPILOT_CORE_%s_C\n\n" name_up;
    (* Print includes and private variables *)
    print_includes (get_includes sm) out_h;
    if has_modules sm then fprintf out_h "\n#include \"modules.h\"\n";
    fprintf out_h "\nuint8_t private_autopilot_mode_%s;\n" name;
    fprintf out_h "uint8_t last_autopilot_mode_%s;\n\n" name;
    (* Print functions *)
    print_ap_init modes name out_h;
    print_test_select modes name out_h;
    print_test_exception modes name out_h;
    print_global_exceptions (get_exceptions sm) name out_h;
    print_set_mode modes name out_h;
    (* Select freq, airframe definition will supersede autopilot one *)
    let freq = match main_freq, (Xml.attrib sm "freq") with
    | None, f -> f
    | Some f, _ -> f
    in
    print_ap_periodic modes ctrl_block freq name out_h;
    (* End and close file *)
    fprintf out_h "\n#endif // AUTOPILOT_CORE_%s_C\n" name_up;
    fprintf out_h "\n#endif // AUTOPILOT_CORE_%s_H\n" name_up;
    close_out out_h;
    (* Move to final destination *)
    let h_file = Filename.concat h_dir (sprintf "autopilot_core_%s.h" name) in
    if Sys.file_exists h_file then Sys.remove h_file;
    if Sys.command (sprintf "mv -f %s %s" tmp_file h_file) > 0 then
      failwith (sprintf "gen_autopilot: fail to move tmp file %s to final location" tmp_file)
  with _ -> Sys.remove tmp_file

(* Output settings xml file *)
let write_settings = fun xml_file out_set ap ->
  fprintf out_set "<!-- This file has been generated by gen_autopilot from %s -->\n" xml_file;
  fprintf out_set "<!-- Version %s -->\n" (Env.get_paparazzi_version ());
  fprintf out_set "<!-- Please DO NOT EDIT -->\n\n";
  fprintf out_set "<settings>\n";
  fprintf out_set "  <dl_settings>\n";
  (* Filter state machines that need to be displayed *)
  let sm_filtered = List.filter (fun sm ->
    try (Compat.lowercase_ascii (Xml.attrib sm "settings_mode")) = "true" with _ -> false
    ) (Xml.children ap) in
  if List.length sm_filtered > 0 then begin
    (* Create node if there is at least one to display *)
    fprintf out_set "    <dl_settings name=\"Autopilot\">\n";
    let write_ap_mode = fun sm ->
      let modes = get_modes sm in
      let name = Xml.attrib sm "name" in
      (* Iter on modes and store min, max and values *)
      let (_, min, max, values) = List.fold_left (fun (current, min, max, values) m ->
        let print = try Compat.lowercase_ascii (Xml.attrib m "settings") <> "hide" with _ -> true in
        let name = Xml.attrib m "name" in
        if print then begin
          let min = match min with
          | None -> Some current
          | Some x -> Some x
          in
          let max = Some current in
          let values = values @ [name] in
          (current + 1, min, max, values)
        end
        else begin
          let n = match min with None -> [] | _ -> [name] in
          (current + 1, min, max, values @ n)
        end
      ) (0, None, None, []) modes in
      (* check handler *)
      let handler = try
        let sh = Xml.attrib sm "settings_handler" in
        let s = Str.split (Str.regexp "|") sh in
        match s with
        | [m; h] -> sprintf " module=\"%s\" handler=\"%s\"" m h
        | _ -> failwith "invalid handler format"
        with _ -> sprintf " module=\"autopilot_core_%s\"" name in
      (* Print if at least one mode has been found *)
      begin match min, max with
      | Some min_idx, Some max_idx ->
          fprintf out_set "      <dl_setting min=\"%d\" max=\"%d\" step=\"1\" var=\"autopilot_mode_%s\" shortname=\"%s\" values=\"%s\"%s/>\n"
          min_idx max_idx name name (String.concat "|" values) handler
      | _, _ -> ()
      end;
      (* check for embedded custom settings *)
      let extra_settings = get_settings sm in
      List.iter (fun s -> fprintf out_set "      %s\n" (Xml.to_string s)) (Xml.children extra_settings)
    in
    (* Iter and call print function *)
    List.iter write_ap_mode sm_filtered;
    fprintf out_set "    </dl_settings>\n";
  end;
  fprintf out_set "  </dl_settings>\n";
  fprintf out_set "</settings>\n"



(** Main generation function
  * Usage: main_freq xml_file_input h_dir_output
  *)
let gen_autopilot main_freq xml_file h_dir out_set =
  let ap_xml =  try Xml.parse_file xml_file with
    | Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
    | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
    | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
    | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
    | Xml.File_not_found _ -> (* invalid file, return empty Xml *)
        Xml.Element ("autopilot", [], [])
  in
  let ap_name = ExtXml.attrib_or_default ap_xml "name" "Autopilot" in
  let state_machines = get_state_machines ap_xml in
  List.iter (parse_and_gen_modes xml_file ap_name main_freq h_dir) state_machines;
  write_settings xml_file out_set ap_xml

(* Main call *)
let () =
  if Array.length Sys.argv <> 4 then
    failwith (Printf.sprintf "Usage: %s airframe_xml_file out_h_dir out_settings" Sys.argv.(0));
  let xml_file = Sys.argv.(1)
  and h_dir = Sys.argv.(2)
  and out_set = open_out Sys.argv.(3) in
  let (autopilot, ap_freq) = try
    let target = try Some (Sys.getenv "TARGET") with _ -> None in
    Gen_common.get_autopilot_of_airframe ?target (Xml.parse_file xml_file)
    with
      Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
    | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
    | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
    | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
    | Not_found -> (* No autopilot file found *)
        ("", None)
  in
  try
    gen_autopilot ap_freq autopilot h_dir out_set;
    close_out out_set
  with
    _ -> fprintf stderr "gen_autopilot: What the heck? Something went wrong...\n"; exit 1
