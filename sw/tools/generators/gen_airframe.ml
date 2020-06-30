(*
 * Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *)

(**
 * XML preprocessing for airframe parameters
 *)

let max_pprz = 9600. (* !!!! MAX_PPRZ From paparazzi.h !!!! *)


open Printf
open Xml2h

type channel = { min : float; max : float; neutral : float }
type control = { failsafe_value : int; foo : int }

let fos = float_of_string
let sof = fun x -> if mod_float x 1. = 0. then Printf.sprintf "%.0f" x else string_of_float x

let servos_drivers = Hashtbl.create 3

let get_servo_driver = fun servo_name ->
  try
    Hashtbl.find servos_drivers servo_name
  with
      Not_found -> failwith (sprintf "gen_airframe, Unknown servo: %s" servo_name)

let get_list_of_drivers = fun () ->
  let l = ref [] in
  Hashtbl.iter
    (fun _s (d, _) -> if not (List.mem d !l) then l := d :: !l)
    servos_drivers;
  !l


let define_integer out name v n =
  let max_val = 1 lsl n in
  let (v,s) = if v >= 0. then (v, 1) else (-.v, -1) in
  let print = fun name num den ->
    define_out out (name^"_NUM") (string_of_int num);
    define_out out (name^"_DEN") (string_of_int den) in
  let rec continious_frac = fun a x num den s ->
    let x1 = 1. /. (x -. (float_of_int a)) in
    let a1 = truncate x1 in
    let (num1, num2) = num in
    let num3 = a1 * num2 + num1 in
    let (den1, den2) = den in
    let den3 = a1 * den2 + den1 in
    if num3 > max_val || den3 > max_val then
      print name num2 (s*den2)
    else if (float_of_int num3) /. (float_of_int den3) = v then
      print name num3 (s*den3)
    else
      continious_frac a1 x1 (num2, num3) (den2, den3) s
  in
  continious_frac (truncate v) v (1, (truncate v)) (0, 1) s

let convert_value_with_code_unit_coef_of_xml = function xml ->
  (* if unit attribute is not specified don't even attempt to convert the units *)
  let u = try Xml.attrib xml "unit" with _ -> failwith "Unit conversion error" in
  let cu = ExtXml.attrib_or_default xml "code_unit" "" in
  (* if unit equals code unit, don't convert as that would always result in a float *)
  if u = cu then failwith "Not converting";
  (* default value for code_unit is rad[/s] when unit is deg[/s] *)
  let conv = try PprzLink.scale_of_units u cu with
  | PprzLink.Unit_conversion_error s ->
      eprintf "Unit conversion error: %s\n%!" s;
      exit 1
  | PprzLink.Unknown_conversion (su, scu) ->
      eprintf "Warning: unknown unit conversion: from %s to %s\n%!" su scu;
      failwith "Unknown unit conversion"
  | PprzLink.No_automatic_conversion _ | _ -> failwith "Unit conversion error" in
  let v =
    try ExtXml.float_attrib xml "value"
    with _ -> prerr_endline (sprintf "Error: Unit conversion of parameter %s impossible because '%s' is not a float" (Xml.attrib xml "name") (Xml.attrib xml "value")); flush stderr; exit 1 in
  v *. conv

let array_sep = Str.regexp "[,;]"
let rec string_from_type = fun name v t ->
  let sprint_array = fun v t ->
    let vs = Str.split array_sep v in
    let sl = List.map (fun vl -> string_from_type name vl t) vs in
    "{ "^(String.concat " , " sl)^" }"
  in
  let rm_leading_trailing_spaces = fun s ->
    let s = Str.global_replace (Str.regexp "^ *") "" s in
    Str.global_replace (Str.regexp " *$") "" s
  in
  match t with
  | "float" ->
      begin
        try
          string_of_float (float_of_string (rm_leading_trailing_spaces v))
        with _ -> prerr_endline (sprintf "Define value %s = %s is not compatible with type float" name v); flush stderr; exit 1
      end
  | "int" ->
      begin
        try
          string_of_int (int_of_string (rm_leading_trailing_spaces v))
        with _ -> prerr_endline (sprintf "Define value %s = %s is not compatible with type int" name v); flush stderr; exit 1
      end
  | "string" -> "\""^(rm_leading_trailing_spaces v)^"\""
  | "array" -> sprint_array v ""
  | "float[]" -> sprint_array v "float"
  | "int[]" -> sprint_array v "int"
  | "string[]" -> sprint_array v "string"
  | _ -> v


let parse_element = fun out prefix s ->
  match Xml.tag s with
      "define" -> begin
        try
          (* fail if units conversion is not found and just copy value instead,
             this is important for integer values, you can't just multiply them with 1.0 *)
          let value =
            try string_of_float (convert_value_with_code_unit_coef_of_xml s)
            with _ -> ExtXml.display_entities (ExtXml.attrib s "value")
          in
          let name = (prefix^ExtXml.attrib s "name") in
          let t = ExtXml.attrib_or_default s "type" "" in
          define_out out name (string_from_type name value t);
          define_integer out name (ExtXml.float_attrib s "value") (ExtXml.int_attrib s "integer");
        with _ -> ();
      end
    | _ -> xml_error "define"


let print_reverse_servo_table = fun out driver servos ->
  let d = match driver with "Default" -> "" | _ -> "_"^(Compat.uppercase_ascii driver) in
  fprintf out "static inline int get_servo_min%s(int _idx) {\n" d;
  fprintf out "  switch (_idx) {\n";
  List.iter (fun c ->
    let name = ExtXml.attrib c "name" in
    fprintf out "    case SERVO_%s: return SERVO_%s_MIN;\n" name name;
  ) servos;
  fprintf out "    default: return 0;\n";
  fprintf out "  };\n";
  fprintf out "}\n\n";
  fprintf out "static inline int get_servo_max%s(int _idx) {\n" d;
  fprintf out "  switch (_idx) {\n";
  List.iter (fun c ->
    let name = ExtXml.attrib c "name" in
    fprintf out "    case SERVO_%s: return SERVO_%s_MAX;\n" name name;
  ) servos;
  fprintf out "    default: return 0;\n";
  fprintf out "  };\n";
  fprintf out "}\n\n"

let parse_servo = fun out driver c ->
  let shortname = ExtXml.attrib c "name" in
  let name = "SERVO_"^shortname
  and no_servo = int_of_string (ExtXml.attrib c "no") in

  define_out out name (string_of_int no_servo);

  let s_min = fos (ExtXml.attrib c "min" )
  and neutral = fos (ExtXml.attrib c "neutral")
  and s_max = fos (ExtXml.attrib c "max" ) in

  let travel_up = (s_max-.neutral) /. max_pprz
  and travel_down = (neutral-.s_min) /. max_pprz in

  define_out out (name^"_NEUTRAL") (sof neutral);
  define_out out (name^"_TRAVEL_UP") (sof travel_up);
  define_integer out (name^"_TRAVEL_UP") travel_up 16;
  define_out out (name^"_TRAVEL_DOWN") (sof travel_down);
  define_integer out (name^"_TRAVEL_DOWN") travel_down 16;

  let s_min = min s_min s_max
  and s_max = max s_min s_max in

  define_out out (name^"_MAX") (sof s_max);
  define_out out (name^"_MIN") (sof s_min);
  fprintf out "\n";

  (* Memorize the associated driver (if any) and global index (insertion order) *)
  let global_idx = Hashtbl.length servos_drivers in
  Hashtbl.add servos_drivers shortname (driver, global_idx)

(* Characters checked in Gen_radio.checl_function_name *)
let pprz_value = Str.regexp "@\\([A-Z_0-9]+\\)"

let var_value = Str.regexp "\\$\\([_a-z0-9]+\\)"
let preprocess_value = fun s v prefix ->
  let s = Str.global_replace pprz_value (sprintf "%s[%s_\\1]" v prefix) s in
  Str.global_replace var_value "_var_\\1" s

let print_actuators_idx = fun out ->
  Hashtbl.iter (fun s (d, i) ->
    fprintf out "#define SERVO_%s_IDX %d\n" s i;
    (* Set servo macro *)
    fprintf out "#define Set_%s_Servo(_v) { \\\n" s;
    fprintf out "  actuators[SERVO_%s_IDX] = Clip(_v, SERVO_%s_MIN, SERVO_%s_MAX); \\\n" s s s;
    fprintf out "  Actuator%sSet(SERVO_%s, actuators[SERVO_%s_IDX]); \\\n" d s s;
    fprintf out "}\n\n"
  ) servos_drivers;
  define_out out "ACTUATORS_NB" (string_of_int (Hashtbl.length servos_drivers));
  fprintf out "\n"

let parse_command_laws = fun out command ->
  let a = fun s -> ExtXml.attrib command s in
  match Xml.tag command with
      "set" ->
        let servo = a "servo"
        and value = a "value" in
        let v = preprocess_value value "values" "COMMAND" in
        fprintf out "  command_value = %s; \\\n" v;
        fprintf out "  command_value *= command_value>0 ? SERVO_%s_TRAVEL_UP_NUM : SERVO_%s_TRAVEL_DOWN_NUM; \\\n" servo servo;
        fprintf out "  command_value /= command_value>0 ? SERVO_%s_TRAVEL_UP_DEN : SERVO_%s_TRAVEL_DOWN_DEN; \\\n" servo servo;
        fprintf out "  servo_value = SERVO_%s_NEUTRAL + command_value; \\\n" servo;
        fprintf out "  Set_%s_Servo(servo_value); \\\n\\\n" servo
    | "let" ->
      let var = a "var"
      and value = a "value" in
      let v = preprocess_value value "values" "COMMAND" in
      fprintf out "  int32_t _var_%s = %s; \\\n" var v
    | "call" ->
      let f = a "fun" in
      fprintf out "  %s; \\\n\\\n" f
    | "ratelimit" ->
      let var = a "var"
      and value = a "value"
      and rate_min = a "rate_min"
      and rate_max = a "rate_max" in
      let v = preprocess_value value "values" "COMMAND" in
      fprintf out "  static int32_t _var_%s = 0; _var_%s += Clip((%s) - (_var_%s), (%s), (%s)); \\\n" var var v var rate_min rate_max
    | "define" ->
      parse_element out "" command
    | _ -> xml_error "set|let"

let parse_rc_commands = fun out rc ->
  let a = fun s -> ExtXml.attrib rc s in
  match Xml.tag rc with
      "set" ->
        let com = a "command"
        and value = a "value" in
        let v = preprocess_value value "_rc_array" "RADIO" in
        fprintf out "  _commands_array[COMMAND_%s] = %s;\\\n" com v;
    | "let" ->
      let var = a "var"
      and value = a "value" in
      let v = preprocess_value value "rc_values" "RADIO" in
      fprintf out "  int16_t _var_%s = %s;\\\n" var v
    | "define" ->
      parse_element out "" rc
    | _ -> xml_error "set|let"

let parse_ap_only_commands = fun out ap_only ->
  let a = fun s -> ExtXml.attrib ap_only s in
  match Xml.tag ap_only with
      "copy" ->
        let com = a "command" in
        fprintf out "  commands[COMMAND_%s] = ap_commands[COMMAND_%s];\\\n" com com
    | _ -> xml_error "copy"

let parse_command = fun out command no ->
  let command_name = "COMMAND_"^ExtXml.attrib command "name" in
  define_out out command_name (string_of_int no);
  let failsafe_value = int_of_string (ExtXml.attrib command "failsafe_value") in
  { failsafe_value = failsafe_value; foo = 0}

let parse_heli_curves = fun out heli_curves ->
  let a = fun s -> ExtXml.attrib heli_curves s in
  match Xml.tag heli_curves with
      "curve" ->
        let throttle = a "throttle" in
        let rpm = ExtXml.attrib_or_default heli_curves "rpm" "" in
        let collective = a "collective" in
        let nb_throttle = List.length (Str.split (Str.regexp ",") throttle) in
        let nb_rpm = List.length (Str.split (Str.regexp ",") rpm) in
        let nb_collective = List.length (Str.split (Str.regexp ",") collective) in
        if nb_throttle < 1 then
          failwith (Printf.sprintf "Need at least one value in throttle curve for a throttle ('%s', '%s')" throttle collective);
        if nb_throttle <> nb_collective then
          failwith (Printf.sprintf "Amount of throttle points not the same as collective in throttle curve ('%s', '%s')" throttle collective);
        if nb_throttle <> nb_rpm && nb_rpm <> 0 then
          failwith (Printf.sprintf "Amount of throttle points not the same as rpm in throttle curve ('%s', '%s', '%s')" throttle collective rpm);
        fprintf out "  {.nb_points = %i, \\\n" nb_throttle;
        fprintf out "   .throttle = {%s}, \\\n" throttle;
        if nb_rpm <> 0 then
          fprintf out "   .rpm = {%s}, \\\n" rpm
        else
          fprintf out "   .rpm = {0xFFFF}, \\\n";
        fprintf out "   .collective = {%s}}, \\\n" collective
    | _ -> xml_error "mixer"

let rec parse_section = fun out ac_id s ->
  match Xml.tag s with
      "section" ->
        let prefix = ExtXml.attrib_or_default s "prefix" "" in
        define_out out ("SECTION_"^ExtXml.attrib s "name") "1";
        List.iter (parse_element out prefix) (Xml.children s);
        fprintf out "\n";
    | "servos" ->
      let driver = ExtXml.attrib_or_default s "driver" "Default" in
      let servos = Xml.children s in
      let nb_servos = List.fold_right (fun s m -> max (int_of_string (ExtXml.attrib s "no")) m) servos min_int + 1 in

      define_out out (sprintf "SERVOS_%s_NB" (Compat.uppercase_ascii driver)) (string_of_int nb_servos);
      fprintf out "#include \"subsystems/actuators/actuators_%s.h\"\n" (Compat.lowercase_ascii driver);
      fprintf out "\n";
      List.iter (parse_servo out driver) servos;
      print_reverse_servo_table out driver servos;
      fprintf out "\n"
    | "commands" ->
      let commands = Array.of_list (Xml.children s) in
      let commands_params = Array.mapi (fun i c -> parse_command out c i) commands in
      define_out out "COMMANDS_NB" (string_of_int (Array.length commands));
      define_out out "COMMANDS_FAILSAFE" (sprint_float_array (List.map (fun x -> string_of_int x.failsafe_value) (Array.to_list commands_params)));
      fprintf out "\n\n"
    | "rc_commands" ->
      fprintf out "#define SetCommandsFromRC(_commands_array, _rc_array) { \\\n";
      List.iter (parse_rc_commands out) (Xml.children s);
      fprintf out "}\n\n"
    | "auto_rc_commands" ->
      fprintf out "#define SetAutoCommandsFromRC(_commands_array, _rc_array) { \\\n";
      List.iter (parse_rc_commands out) (Xml.children s);
      fprintf out "}\n\n"
    | "ap_only_commands" ->
      fprintf out "#define SetApOnlyCommands(ap_commands) { \\\n";
      List.iter (parse_ap_only_commands out) (Xml.children s);
      fprintf out "}\n\n"
    | "command_laws" ->
      (* print actuators index and set macros *)
      print_actuators_idx out;
      (* print init and commit actuators macros *)
      let drivers = get_list_of_drivers () in
      fprintf out "#define AllActuatorsInit() { \\\n";
      List.iter (fun d -> fprintf out "  Actuators%sInit();\\\n" d) drivers;
      fprintf out "}\n\n";
      fprintf out "#define AllActuatorsCommit() { \\\n";
      List.iter (fun d -> fprintf out "  Actuators%sCommit();\\\n" d) drivers;
      fprintf out "}\n\n";
      (* print actuators from commands macro *)
      fprintf out "#define SetActuatorsFromCommands(values, AP_MODE) { \\\n";
      fprintf out "  int32_t servo_value;\\\n";
      fprintf out "  int32_t command_value;\\\n\\\n";
      List.iter (parse_command_laws out) (Xml.children s);
      fprintf out "  AllActuatorsCommit(); \\\n";
      fprintf out "}\n\n";
    | "heli_curves" ->
      let default = ExtXml.attrib_or_default s "default" "0" in
      let curves = Xml.children s in
      let nb_points = List.fold_right (fun s m -> max (List.length (Str.split (Str.regexp ",") (ExtXml.attrib s "throttle"))) m) curves 0 in
      define_out out "THROTTLE_CURVE_MODE_INIT" default;
      define_out out "THROTTLE_CURVES_NB" (string_of_int (List.length curves));
      define_out out "THROTTLE_POINTS_NB" (string_of_int nb_points);
      fprintf out "#define THROTTLE_CURVES { \\\n";
      List.iter (parse_heli_curves out) curves;
      fprintf out "}\n\n";
    | "include" ->
      let filename = Str.global_replace (Str.regexp "\\$AC_ID") ac_id (ExtXml.attrib s "href") in
      let subxml = ExtXml.parse_file filename in
      fprintf out "/* XML %s */" filename;
      fprintf out "\n";
      List.iter (parse_section out ac_id) (Xml.children subxml)
    | "makefile" ->
      ()
  (** Ignoring this section *)
    | _ -> ()


let h_name = "AIRFRAME_H"

let hex_to_bin = fun s ->
  let n = String.length s in
  assert(n mod 2 = 0);
  let b = Bytes.make (2*n) 'x' in
  for i = 0 to n/2 - 1 do
    Bytes.set b (4*i) '\\';
    Scanf.sscanf (String.sub s (2*i) 2) "%2x"
      (fun x ->
        Bytes.blit_string (sprintf "%03o" x) 0 b (4*i+1) 3)
  done;
  Bytes.to_string b

let generate = fun airframe ac_id ac_name md5sum xml_file out_file ->
  let out = open_out out_file in

  begin_out out xml_file h_name;
  define_string_out out "AIRFRAME_NAME" ac_name;
  define_out out "AC_ID" ac_id;
  define_out out "MD5SUM" (sprintf "((uint8_t*)\"%s\")" (hex_to_bin md5sum));
  fprintf out "\n";
  List.iter (parse_section out ac_id) (Xml.children airframe.Airframe.xml);
  finish_out out h_name

