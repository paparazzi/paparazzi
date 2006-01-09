(*
 * $Id$
 *
 * XML preprocessing for airframe parameters
 *  
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

let max_pprz = 600. (* !!!! MAX_PPRZ From link_autopilot.h !!!! *)
(* let nb_servo_4017 = 10  From servo.h *)

open Printf
open Xml2h


type channel = { min : float; max : float; neutral : float }
type control = { failsafe_value : int; foo : int }

let fos = float_of_string
let sof = fun x -> if mod_float x 1. = 0. then Printf.sprintf "%.0f" x else string_of_float x
let soi = string_of_int

let define_macro name n x =
  let a = fun s -> ExtXml.attrib x s in
  printf "#define %s(" name;
  match n with   (* Do we really need more ??? *)
    1 -> printf "x1) (%s*(x1))\n" (a "coeff1")
  | 2 -> printf "x1,x2) (%s*(x1)+ %s*(x2))\n" (a "coeff1") (a "coeff2")
  | 3 -> printf "x1,x2,x3) (%s*(x1)+ %s*(x2)+%s*(x3))\n" (a "coeff1") (a "coeff2") (a "coeff3")
  | _ -> failwith "define_macro"
	  
let parse_element = fun prefix s ->
  match Xml.tag s with
    "define" -> begin
			try	
      	define (prefix^ExtXml.attrib s "name") (ExtXml.attrib s "value");
      	define (prefix^(ExtXml.attrib s "name")^"_NB_SAMPLE") (ExtXml.attrib s "nb_sample");
			with _ -> ();
		end
  | "linear" ->
      let name = ExtXml.attrib s "name"
      and n = int_of_string (ExtXml.attrib s "arity") in
      define_macro (prefix^name) n s
  | _ -> xml_error "define|linear"


let parse_servo = fun c ->
  let name = "SERVO_"^ExtXml.attrib c "name" in
  let no_servo = int_of_string (ExtXml.attrib c "no") in
  define name (string_of_int no_servo);
  let min = fos (ExtXml.attrib c "min" )
  and neutral = fos (ExtXml.attrib c "neutral")
  and max = fos (ExtXml.attrib c "max" ) in
  
  let travel_up = (max-.neutral) /. max_pprz
  and travel_down = (neutral-.min) /. max_pprz in
  define (name^"_NEUTRAL") (sof neutral);
  define (name^"_TRAVEL_UP") (sof travel_up);
  define (name^"_TRAVEL_DOWN") (sof travel_down);
  define (name^"_MAX") (sof max);
  define (name^"_MIN") (sof min);
  nl ()

(* Characters checked in Gen_radio.checl_function_name *)
let pprz_value = Str.regexp "@\\([A-Z_0-9]+\\)"

let var_value = Str.regexp "\\$\\([_a-z0-9]+\\)"
let preprocess_command = fun s ->
  let s = Str.global_replace pprz_value "values[COMMAND_\\1]" s in
  Str.global_replace var_value "_var_\\1" s

let parse_command_laws = fun command ->
  let a = fun s -> ExtXml.attrib command s in
   match Xml.tag command with
     "set" ->
       let servo = a "servo"
       and value = a "value" in
       let v = preprocess_command value in
       printf "  command_value = %s;\\\n" v;
       printf "  command_value *= command_value>0 ? SERVO_%s_TRAVEL_UP : SERVO_%s_TRAVEL_DOWN;\\\n" servo servo;
       printf "  servo_value = SERVO_%s_NEUTRAL + (int16_t)(command_value);\\\n" servo;
       printf "  COMMAND(SERVO_%s) = SYS_TICS_OF_USEC(Chop(servo_value, SERVO_%s_MIN, SERVO_%s_MAX));\\\n\\\n" servo servo servo
   | "let" ->
       let var = a "var"
       and value = a "value" in
       let v = preprocess_command value in
       printf "  int16_t _var_%s = %s;\\\n" var v 
   | "define" ->
       parse_element "" command
   | _ -> xml_error "set|let"


let parse_command = fun commands_params command no ->
   let command_name = "COMMAND_"^ExtXml.attrib command "name" in
   let failsafe_value = int_of_string (ExtXml.attrib command "failsafe_value") in
   commands_params.(no) <- { failsafe_value = failsafe_value; foo = 0};
   define command_name (string_of_int no);
   no+1

let parse_section = fun s ->
  match Xml.tag s with
    "section" ->
      let prefix = ExtXml.attrib_or_default s "prefix" "" in
      define ("SECTION_"^ExtXml.attrib s "name") "1";
      List.iter (parse_element prefix) (Xml.children s);
      nl ()
  | "servos" ->
      let servos = Xml.children s in
      let nb_servos = List.fold_right (fun s m -> Pervasives.max (int_of_string (ExtXml.attrib s "no")) m) servos min_int in
      define "LAST_SERVO_CHANNEL" (string_of_int nb_servos);
      nl ();
      List.iter parse_servo servos;
      nl ()
  | "commands" ->
      let commands = Xml.children s in
      let commands_params = Array.create (List.length commands) { failsafe_value = 0; foo = 0} in
      let nb_commands = List.fold_right (parse_command commands_params) commands 0 in
      define "COMMANDS_NB" (string_of_int nb_commands);
      let commands_params = Array.to_list commands_params in
      define "COMMANDS_FAILSAFE" (sprint_float_array (List.map (fun x -> string_of_int x.failsafe_value) commands_params));
      nl (); nl ()
 | "command_laws" ->
      printf "#define CommandsSet(values) { \\\n";
      printf "  uint16_t servo_value;\\\n";
      printf "  float command_value;\\\n";
      List.iter parse_command_laws (Xml.children s);
      printf "}\n"
  | "makefile" ->
      ()
      (** Ignoring this section *)
  | _ -> xml_error (Printf.sprintf "[%s] section|servos|command_laws|conmmands|makefile" (Xml.tag s))
      

let h_name = "AIRFRAME_H"

let _ =
  if Array.length Sys.argv <> 4 then
    failwith (Printf.sprintf "Usage: %s A/C_ident A/C_name xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(3)
  and ac_id = Sys.argv.(1)
  and ac_name = Sys.argv.(2) in
  try
    let xml = start_and_begin xml_file h_name in
    Xml2h.warning ("AIRFRAME MODEL: "^ ac_name);
    define_string "AIRFRAME_NAME" ac_name;
    define "AC_ID" ac_id;
    nl ();
    List.iter parse_section (Xml.children xml);
    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
	  
