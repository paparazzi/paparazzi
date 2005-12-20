(*
 * $Id$
 *
 * XML preprocessing for in flight calibration
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

(** Generates code for tuning parameters using RC sliders *)

open Printf
open Xml2h

let margin = ref 0
let step = 2
let tab () = printf "%s" (String.make !margin ' ')
let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun f -> tab (); printf f


let calib_mode_of_rc = function
    "gain_1_up" -> 1, "up"
  | "gain_1_down" -> 1, "down"
  | "gain_2_up" -> 2, "up"
  | "gain_2_down" -> 2, "down"
  | x -> failwith (sprintf "Unknown rc: %s" x)

let param_macro_of_type = fun x -> "ParamVal"^String.capitalize x

let inttype = function
    "int16" -> "int16_t"
  | "float" -> "float"
  | x -> failwith (sprintf "Gen_calib.inttype: unknown type '%s'" x)

let parse_setting = fun xml ->
  let cursor, cm = calib_mode_of_rc (ExtXml.attrib xml "rc")
  and var = ExtXml.attrib xml "var"
  and range = float_of_string (ExtXml.attrib xml "range") in
  let t = (ExtXml.attrib xml "type") in
  let param_macro = param_macro_of_type t in
  let var_init = var ^ "_init" in

  lprintf "if (inflight_calib_mode == IF_CALIB_MODE_%s) {\n" (String.uppercase cm);
  right ();
  lprintf "static %s %s;\n" (inttype t) var_init;
  lprintf "static int16_t slider%d_init;\n" cursor;
  lprintf "if (mode_changed) {\n";
  right ();
  lprintf "%s = %s;\n" var_init var;
  lprintf "slider%d_init = from_fbw.channels[RADIO_GAIN%d];\n" cursor cursor;
  left (); lprintf "}\n";
  lprintf "%s = %s(%s, %f, from_fbw.channels[RADIO_GAIN%d], slider%d_init);\n" var param_macro var_init range cursor cursor;
  lprintf "slider_%d_val = (float)%s;\n" cursor var;
  left (); lprintf "}\n"


let parse_mode = fun xml ->
  lprintf "if (pprz_mode == PPRZ_MODE_%s) {\n" (ExtXml.attrib xml "name");
  right ();
  List.iter parse_setting (Xml.children xml);
  left (); lprintf "}\n"

let parse_modes = fun xml ->
  List.iter parse_mode (Xml.children xml)


let _ =
  if Array.length Sys.argv < 2 then
    failwith (Printf.sprintf "Usage: %s xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(1) in
  let h_name = "INFLIGHT_CALIB_H" in
  try
    let xml = start_and_begin xml_file h_name in

    let rc_control = try ExtXml.child xml "rc_control" with Not_found -> failwith (sprintf "Error: 'rc_control' child expected in %s" (Xml.to_string xml)) in
    lprintf "void inflight_calib(bool_t mode_changed) {\n";
    right ();
    parse_modes rc_control;
    left (); lprintf "}\n";

    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
  | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
