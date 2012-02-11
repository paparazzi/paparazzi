(*
 * $Id$
 *
 * XML preprocessing for airframe parameters
 *
 * Copyright (C) 2003-2006 Pascal Brisset, Antoine Drouin
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
    (fun _s d -> if not (List.mem d !l) then l := d :: !l)
    servos_drivers;
  !l


let define_macro name n x =
  let a = fun s -> ExtXml.attrib x s in
  printf "#define %s(" name;
  match n with   (* Do we really need more ??? *)
    1 -> printf "x1) (%s*(x1))\n" (a "coeff1")
  | 2 -> printf "x1,x2) (%s*(x1)+ %s*(x2))\n" (a "coeff1") (a "coeff2")
  | 3 -> printf "x1,x2,x3) (%s*(x1)+ %s*(x2)+%s*(x3))\n" (a "coeff1") (a "coeff2") (a "coeff3")
  | _ -> failwith "define_macro"

let define_integer name v n =
  let max_val = 1 lsl n in
  let print = fun name num den ->
    define (name^"_NUM") (string_of_int num);
    define (name^"_DEN") (string_of_int den) in
  let rec continious_frac = fun a x num den ->
    let x1 = 1. /. (x -. (float_of_int a)) in
    let a1 = truncate x1 in
    let (num1, num2) = num in
    let num3 = a1 * num2 + num1 in
    let (den1, den2) = den in
    let den3 = a1 * den2 + den1 in
    if num3 > max_val || den3 > max_val then
      print name num2 den2
    else if (float_of_int num3) /. (float_of_int den3) = v then
      print name num3 den3
    else
      continious_frac a1 x1 (num2, num3) (den2, den3)
  in
  continious_frac (truncate v) v (1, (truncate v)) (0, 1)

let code_unit_scale_of_tag = function t ->
  (* if unit attribute is not specified don't even attempt to convert the units *)
  let u = try ExtXml.attrib t "unit" with _ -> failwith "Unit conversion error" in
  let cu = try ExtXml.attrib t "code_unit" with _ -> "" in
  (* default value for code_unit is rad[/s] when unit is deg[/s] *)
  try match (u, cu) with
      ("deg", "") -> Pprz.scale_of_units u "rad" (* implicit conversion to rad *)
    | ("deg/s", "") -> Pprz.scale_of_units u "rad/s" (* implicit conversion to rad/s *)
    | (_, "") -> failwith "Unit conversion error" (* code unit is not defined and no implicit conversion *)
    | (_,_) -> Pprz.scale_of_units u cu (* try to convert *)
  with
      Pprz.Unit_conversion_error s -> prerr_endline (sprintf "Unit conversion error: %s" s); flush stderr; exit 1
    | Pprz.Unknown_conversion (su, scu) -> prerr_endline (sprintf "Warning: unknown unit conversion: from %s to %s" su scu); flush stderr; failwith "Unknown unit conversion"
    | _ -> failwith "Unit conversion error"


let parse_element = fun prefix s ->
  match Xml.tag s with
      "define" -> begin
        try
          begin
            (* fail if units conversion is not found and just copy value instead,
               this is important for integer values, you can't just multiply them with 1.0 *)
            try
              let value = (ExtXml.float_attrib s "value") *. (code_unit_scale_of_tag s) in
              define (prefix^ExtXml.attrib s "name") (string_of_float value);
            with
                _ -> define (prefix^ExtXml.attrib s "name") (ExtXml.display_entities (ExtXml.attrib s "value"));
          end;
          define_integer (prefix^(ExtXml.attrib s "name")) (ExtXml.float_attrib s "value") (ExtXml.int_attrib s "integer");
        with _ -> ();
      end
    | "linear" ->
      let name = ExtXml.attrib s "name"
      and n = int_of_string (ExtXml.attrib s "arity") in
      define_macro (prefix^name) n s
    | _ -> xml_error "define|linear"



let parse_servo = fun driver c ->
  let shortname = ExtXml.attrib c "name" in
  let name = "SERVO_"^shortname
  and no_servo = int_of_string (ExtXml.attrib c "no") in

  define name (string_of_int no_servo);

  let min = fos (ExtXml.attrib c "min" )
  and neutral = fos (ExtXml.attrib c "neutral")
  and max = fos (ExtXml.attrib c "max" ) in

  let travel_up = (max-.neutral) /. max_pprz
  and travel_down = (neutral-.min) /. max_pprz in

  define (name^"_NEUTRAL") (sof neutral);
  define (name^"_TRAVEL_UP") (sof travel_up);
  define (name^"_TRAVEL_DOWN") (sof travel_down);

  let min = Pervasives.min min max
  and max = Pervasives.max min max in

  define (name^"_MAX") (sof max);
  define (name^"_MIN") (sof min);
  nl ();

  (* Memorize the associated driver (if any) *)
  Hashtbl.add servos_drivers shortname driver

(* Characters checked in Gen_radio.checl_function_name *)
let pprz_value = Str.regexp "@\\([A-Z_0-9]+\\)"

let var_value = Str.regexp "\\$\\([_a-z0-9]+\\)"
let preprocess_value = fun s v prefix ->
  let s = Str.global_replace pprz_value (sprintf "%s[%s_\\1]" v prefix) s in
  Str.global_replace var_value "_var_\\1" s



let parse_command_laws = fun command ->
  let a = fun s -> ExtXml.attrib command s in
   match Xml.tag command with
     "set" ->
       let servo = a "servo"
       and value = a "value" in
       let v = preprocess_value value "values" "COMMAND" in
       printf "  command_value = %s;\\\n" v;
       printf "  command_value *= command_value>0 ? SERVO_%s_TRAVEL_UP : SERVO_%s_TRAVEL_DOWN;\\\n" servo servo;
       printf "  servo_value = SERVO_%s_NEUTRAL + (int32_t)(command_value);\\\n" servo;
       printf "  actuators[SERVO_%s] = ChopServo(servo_value, SERVO_%s_MIN, SERVO_%s_MAX);\\\n\\\n" servo servo servo;

       let driver = get_servo_driver servo in
       printf "  Actuator%s(SERVO_%s) = SERVOS_TICS_OF_USEC(actuators[SERVO_%s]);\\\n\\\n" driver servo servo
   | "let" ->
       let var = a "var"
       and value = a "value" in
       let v = preprocess_value value "values" "COMMAND" in
       printf "  int16_t _var_%s = %s;\\\n" var v
   | "define" ->
       parse_element "" command
   | _ -> xml_error "set|let"

let parse_csc_fields = fun csc_fields ->
  let a = fun s -> ExtXml.attrib csc_fields s in
   match Xml.tag csc_fields with
     "field_map" ->
       let servo_id = a "servo_id"
       and field = a "field" in
       printf "  temp.%s = actuators[%s]; \\\n" field servo_id;
      | _ -> xml_error "field_map"

let parse_csc_messages = (let msg_index_ref = ref 0 in fun csc_id csc_messages ->
  let a = fun s -> ExtXml.attrib csc_messages s in
   match Xml.tag csc_messages with
     "msg" ->
       let msg_id = a "id"
       and msg_type = a "type"
       and msg_index = msg_index_ref.contents in
       msg_index_ref.contents <- msg_index + 1;
       printf "{\\\n  struct Csc%s temp; \\\n" msg_type;
       List.iter parse_csc_fields (Xml.children csc_messages);
       printf "  can_write_csc(%s, CSC_%s, (uint8_t *)&temp, sizeof(struct Csc%s)); \\\n" csc_id msg_id msg_type;
       printf "} \\\n"
      | _ -> xml_error "msg"
  )

let parse_csc_boards = fun csc_board ->
  let a = fun s -> ExtXml.attrib csc_board s in
   match Xml.tag csc_board with
     "board" ->
       let csc_id = a "id" in
       List.iter (parse_csc_messages csc_id) (Xml.children csc_board);
   | "define" ->
       parse_element "" csc_board
   | _ -> xml_error "board"

let parse_rc_commands = fun rc ->
  let a = fun s -> ExtXml.attrib rc s in
  match Xml.tag rc with
    "set" ->
      let com = a "command"
      and value = a "value" in
      let v = preprocess_value value "_rc_array" "RADIO" in
      printf "  _commands_array[COMMAND_%s] = %s;\\\n" com v;
   | "let" ->
       let var = a "var"
       and value = a "value" in
       let v = preprocess_value value "rc_values" "RADIO" in
       printf "  int16_t _var_%s = %s;\\\n" var v
   | "define" ->
       parse_element "" rc
   | _ -> xml_error "set|let"

let parse_ap_only_commands = fun ap_only ->
  let a = fun s -> ExtXml.attrib ap_only s in
  match Xml.tag ap_only with
    "copy" ->
      let com = a "command" in
      printf "  commands[COMMAND_%s] = ap_commands[COMMAND_%s];\\\n" com com
   | _ -> xml_error "copy"

let parse_command = fun command no ->
   let command_name = "COMMAND_"^ExtXml.attrib command "name" in
   define command_name (string_of_int no);
   let failsafe_value = int_of_string (ExtXml.attrib command "failsafe_value") in
   { failsafe_value = failsafe_value; foo = 0}

let rec parse_section = fun s ->
  match Xml.tag s with
    "section" ->
      let prefix = ExtXml.attrib_or_default s "prefix" "" in
      define ("SECTION_"^ExtXml.attrib s "name") "1";
      List.iter (parse_element prefix) (Xml.children s);
      nl ()
  | "servos" ->
      let driver = ExtXml.attrib_or_default s "driver" "" in
      let servos = Xml.children s in
      let nb_servos = List.fold_right (fun s m -> Pervasives.max (int_of_string (ExtXml.attrib s "no")) m) servos min_int + 1 in

      define "SERVOS_NB" (string_of_int nb_servos);
      nl ();
      List.iter (parse_servo driver) servos;
      nl ()
  | "commands" ->
      let commands = Array.of_list (Xml.children s) in
      let commands_params = Array.mapi (fun i c -> parse_command c i) commands in
      define "COMMANDS_NB" (string_of_int (Array.length commands));
      define "COMMANDS_FAILSAFE" (sprint_float_array (List.map (fun x -> string_of_int x.failsafe_value) (Array.to_list commands_params)));
      nl (); nl ()
  | "rc_commands" ->
      printf "#define SetCommandsFromRC(_commands_array, _rc_array) { \\\n";
      List.iter parse_rc_commands (Xml.children s);
      printf "}\n\n"
  | "auto_rc_commands" ->
      printf "#define SetAutoCommandsFromRC(_commands_array, _rc_array) { \\\n";
      List.iter parse_rc_commands (Xml.children s);
      printf "}\n\n"
  | "ap_only_commands" ->
      printf "#define SetApOnlyCommands(ap_commands) { \\\n";
      List.iter parse_ap_only_commands (Xml.children s);
      printf "}\n\n"
  | "command_laws" ->
      printf "#define SetActuatorsFromCommands(values) { \\\n";
      printf "  uint32_t servo_value;\\\n";
      printf "  float command_value;\\\n";

      List.iter parse_command_laws (Xml.children s);

      let drivers = get_list_of_drivers () in
      List.iter (fun d -> printf "  Actuators%sCommit();\\\n" d) drivers;
      printf "}\n\n";

      printf "#define AllActuatorsInit() { \\\n";
      List.iter (fun d -> printf "  Actuators%sInit();\\\n" d) drivers;
      printf "}\n\n";
  | "csc_boards" ->
      let boards = Array.of_list (Xml.children s) in
      define "CSC_BOARD_NB" (string_of_int (Array.length boards));
      printf "#define SendCscFromActuators() { \\\n";
      List.iter parse_csc_boards (Xml.children s);
      printf "}\n"
  | "include" ->
      let filename = ExtXml.attrib s "href" in
      let subxml = Xml.parse_file filename in
      printf "/* XML %s */" filename;
      nl ();
      List.iter parse_section (Xml.children subxml)
  | "makefile" ->
      ()
      (** Ignoring this section *)
  | _ -> ()


let h_name = "AIRFRAME_H"

let hex_to_bin = fun s ->
  let n = String.length s in
  assert(n mod 2 = 0);
  let b = String.make (2*n) 'x' in
  for i = 0 to n/2 - 1 do
    b.[4*i] <- '\\';
    Scanf.sscanf (String.sub s (2*i) 2) "%2x"
      (fun x ->
    String.blit (sprintf "%03o" x) 0 b (4*i+1) 3)
  done;
  b

let _ =
  if Array.length Sys.argv <> 5 then
    failwith (Printf.sprintf "Usage: %s A/C_ident A/C_name MD5SUM xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(4)
  and ac_id = Sys.argv.(1)
  and ac_name = Sys.argv.(2)
  and md5sum = Sys.argv.(3) in
  try
    let xml = start_and_begin xml_file h_name in
    Xml2h.warning ("AIRFRAME MODEL: "^ ac_name);
    define_string "AIRFRAME_NAME" ac_name;
    define "AC_ID" ac_id;
    define "MD5SUM" (sprintf "((uint8_t*)\"%s\")" (hex_to_bin md5sum));
    nl ();
    List.iter parse_section (Xml.children xml);
    finish h_name
  with
    Xml.Error e -> fprintf stderr "%s: XML error:%s\n" xml_file (Xml.error e); exit 1
  | Dtd.Prove_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.prove_error e); exit 1
  | Dtd.Check_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.check_error e); exit 1
  | Dtd.Parse_error e -> fprintf stderr "%s: DTD error:%s\n%!" xml_file (Dtd.parse_error e); exit 1
