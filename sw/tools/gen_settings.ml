(*
 * $Id$
 *
 * XML preprocessing for dynamic tuning
 *  
 * Copyright (C) 2006 Pascal Brisset, Antoine Drouin
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

(** Generates code for tuning parameters using RC sliders and datalink *)

open Printf
open Xml2h

let margin = ref 0
let step = 2
let tab () = printf "%s" (String.make !margin ' ')
let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun f -> tab (); printf f

let rec flatten = fun xml r ->
  if ExtXml.tag_is xml "dl_setting" then
    xml::r
  else
    match Xml.children xml with
      [] -> r
    | x::xs ->
	List.iter (fun y -> assert(ExtXml.tag_is y (Xml.tag x))) xs;
	List.fold_right flatten (x::xs) r
	  

let print_dl_settings = fun settings ->
  let settings = flatten settings [] in
    (** include  headers **)
    (** FIXME : add header only once **)
    printf "\n";
    List.iter 
    (fun s ->
       try
	 let v = ExtXml.attrib s "module" in
	   printf "#include \"%s.h\"\n" v
       with  ExtXml.Error e -> ()
    ) 
      settings;
    printf "\n";
    
  (** Macro to call to set one variable *)
  lprintf "#define DlSetting(_idx, _value) { \\\n";
  right ();
  lprintf "switch (_idx) { \\\n";
  right ();
  let idx = ref 0 in
  List.iter 
    (fun s ->
      let v = ExtXml.attrib s "var" in
	begin
	try
	  let h = ExtXml.attrib s "handler" and
	      m =  ExtXml.attrib s "module" in
	    lprintf "case %d: %s_%s( _value ); break;\\\n" !idx m h 
	with  
	    ExtXml.Error e -> lprintf "case %d: %s = _value; break;\\\n" !idx v
	end;
	incr idx
    ) 
    settings;
  left ();
  lprintf "}\\\n";
  left ();
  lprintf "}\n";
  let nb_values = !idx in

  (** Macro to call to downlink current values *)
  lprintf "#define PeriodicSendDlValue() { \\\n";
  if nb_values > 0 then begin
    right ();
    lprintf "static uint8_t i;\\\n";
    lprintf "float var;\\\n";
    lprintf "if (i >= %d) i = 0;;\\\n" nb_values;
    let idx = ref 0 in
    lprintf "switch (i) { \\\n";
    right ();
    List.iter 
      (fun s ->
	let v = ExtXml.attrib s "var" in
	lprintf "case %d: var = %s; break;\\\n" !idx v; incr idx) 
      settings;
    lprintf "default: var = 0.; break;\\\n";
    left ();
    lprintf "}\\\n";
    lprintf "DOWNLINK_SEND_DL_VALUE(&i, &var);\\\n";
    lprintf "i++;\\\n";
    left ()
  end;
  lprintf "}\n"




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

let parse_rc_setting = fun xml ->
  let cursor, cm = calib_mode_of_rc (ExtXml.attrib xml "rc")
  and var = ExtXml.attrib xml "var"
  and range = float_of_string (ExtXml.attrib xml "range") in
  let t = (ExtXml.attrib xml "type") in
  let param_macro = param_macro_of_type t in
  let var_init = var ^ "_init" in

  lprintf "if (rc_settings_mode == RC_SETTINGS_MODE_%s) { \\\n" (String.uppercase cm);
  right ();
  lprintf "static %s %s; \\\n" (inttype t) var_init;
  lprintf "static int16_t slider%d_init; \\\n" cursor;
  lprintf "if (mode_changed) { \\\n";
  right ();
  lprintf "%s = %s; \\\n" var_init var;
  lprintf "slider%d_init = RcChannel(RADIO_GAIN%d); \\\n" cursor cursor;
  left (); lprintf "} \\\n";
  lprintf "%s = %s(%s, %f, RcChannel(RADIO_GAIN%d), slider%d_init); \\\n" var param_macro var_init range cursor cursor;
  lprintf "slider_%d_val = (float)%s; \\\n" cursor var;
  left (); lprintf "} \\\n"


let parse_rc_mode = fun xml ->
  lprintf "if (pprz_mode == PPRZ_MODE_%s) { \\\n" (ExtXml.attrib xml "name");
  right ();
  List.iter parse_rc_setting (Xml.children xml);
  left (); lprintf "} \\\n"

let parse_rc_modes = fun xml ->
  List.iter parse_rc_mode (Xml.children xml)


let _ =
  if Array.length Sys.argv < 2 then
    failwith (Printf.sprintf "Usage: %s xml_file" Sys.argv.(0));
  let xml_file = Sys.argv.(1) in
  let h_name = "SETTINGS_H" in
  try
    let xml = start_and_begin xml_file h_name in

    let rc_control = 
      try ExtXml.child xml "rc_settings" with Not_found -> Xml.Element("",[],[]) in
    lprintf "#define RCSettings(mode_changed) { \\\n";
    right ();
    parse_rc_modes rc_control;
    left (); lprintf "}\n";


    let dl_settings = try (ExtXml.child xml "dl_settings") with Not_found -> Xml.Element("dl_settings",[],[]) in

    print_dl_settings dl_settings;
    

    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e)
  | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
  | Dtd.Parse_error e ->  prerr_endline (Dtd.parse_error e); exit 1
