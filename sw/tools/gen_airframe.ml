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

let command_travel = 1200. (* !!!! From link_autopilot.h !!!! *)
let nb_servo_4017 = 10 (* From servo.h *)

open Printf
open Xml2h


type channel = { min : float; max : float; neutral : float }

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
    "define" ->
      define (prefix^ExtXml.attrib s "name") (ExtXml.attrib s "value")
  | "linear" ->
      let name = ExtXml.attrib s "name"
      and n = int_of_string (ExtXml.attrib s "arity") in
      define_macro (prefix^name) n s
  | _ -> xml_error "define|linear"


let parse_servo =
  fun default_min default_neutral default_max servo_params c ->
    let name = "SERVO_"^ExtXml.attrib c "name" in
    let no_servo = int_of_string (ExtXml.attrib c "no") in
    define name (string_of_int no_servo);
    let min = fos (ExtXml.attrib_or_default c "min" (sof default_min))
    and neutral = fos (ExtXml.attrib_or_default c "neutral" (sof default_neutral))
    and max = fos (ExtXml.attrib_or_default c "max" (sof default_max)) in
    
    let travel = (max-.min) /. command_travel in
    define (name^"_TRAVEL") (sof travel);
    define (sprintf "SERVOS_NEUTRALS_%d" no_servo) (sof neutral);
    nl ();
    
    servo_params.(no_servo) <- { min = min; neutral = neutral; max = max }


let pprz_value = Str.regexp "@\\([A-Z]+\\)"
let var_value = Str.regexp "\\$\\([_a-z0-9]+\\)"
let preprocess_command = fun s ->
  let s = Str.global_replace pprz_value "values[RADIO_\\1]" s in
  Str.global_replace var_value "_var_\\1" s

let parse_command = fun command ->
  let a = fun s -> ExtXml.attrib command s in
   match Xml.tag command with
     "set" ->
       let servo = a "servo"
       and value = a "value" in
       let v = preprocess_command value in
       printf "  servo_value = SERVO_NEUTRAL(SERVO_%s) + (int16_t)((%s)*SERVO_%s_TRAVEL);\\\n" servo v servo;
       printf "  servo_widths[SERVO_%s] = ChopServo(servo_value);\\\n\\\n" servo
   | "let" ->
       let var = a "var"
       and value = a "value" in
       let v = preprocess_command value in
       printf "  int16_t _var_%s = %s;\\\n" var v       
   | _ -> xml_error "set|let"

let parse_section = fun s ->
  match Xml.tag s with
    "section" ->
      let prefix = ExtXml.attrib_or_default s "prefix" "" in
      List.iter (parse_element prefix) (Xml.children s);
      nl ()
  | "servos" ->
      let get_float = fun x -> float_of_string (ExtXml.attrib s x) in
      let min = get_float "min"
      and neutral = get_float "neutral"
      and max = get_float "max" in

      let servos = Xml.children s in
      define "NB_SERVO" (string_of_int (List.length servos));
      nl ();
      let servos_params = Array.create nb_servo_4017 { min = min; neutral = neutral; max = max } in

      List.iter (parse_servo min neutral max servos_params) servos;

      let servos_params = Array.to_list servos_params in
      
      nl ();
      define "SERVOS_MINS" (sprint_float_array (List.map (fun x -> sof x.min) servos_params));
      define "SERVOS_NEUTRALS" (sprint_float_array (List.map (fun x -> sof x.neutral) servos_params));
      define "SERVOS_MAXS" (sprint_float_array (List.map (fun x -> sof x.max) servos_params));
      nl ();

      (* For compatibility *)
      define "SERVO_MIN_US" (sprintf "%.0ful" min);
      define "SERVO_MAX_US" (sprintf "%.0ful" max);
      nl ()
  | "command" ->
      printf "#define ServoSet(values) { \\\n";
      printf "  uint16_t servo_value;\\\n";
      List.iter parse_command (Xml.children s);
      printf "}\n"
  | _ -> xml_error "param|servos|command"
      

let h_name = "AIRFRAME_H"

let _ =
  if Array.length Sys.argv <> 3 then
    failwith (Printf.sprintf "Usage: %s A/C_ident xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(2)
  and ac_name = Sys.argv.(1) in
  try
    let xml = start_and_begin xml_file h_name in
    Xml2h.warning ("AIRFRAME MODEL: "^ ac_name);
    define_string "AIRFRAME_NAME" ac_name;
    nl ();
    let v = ExtXml.attrib xml "ctl_board" in
    define ("CTL_BRD_"^v) "1";
    nl ();
    List.iter parse_section (Xml.children xml);
    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
	  
