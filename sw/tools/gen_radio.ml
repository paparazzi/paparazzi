(*
 * $Id$
 *
 * XML preprocessing for radio-control parameters
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

open Printf
open Xml2h

let h_name = "RADIO_H"

let fos = float_of_string

type channel = { min : string; max : string; neutral : string; averaged : string }

let default_neutral = "1600"
let default_min = "1000"
let default_max = "2200"

let parse_channel =
  let no_channel = ref 0 in
  fun c ->
    let ctl = "RADIO_CTL_"^ExtXml.attrib c "ctl"
    and fct = "RADIO_" ^ ExtXml.attrib c "function" in
    define ctl (string_of_int !no_channel);
    define fct ctl;
    no_channel := !no_channel + 1;
    { min = ExtXml.attrib_or_default c "min" default_min;
      neutral = ExtXml.attrib_or_default c "neutral" default_neutral;
      max = ExtXml.attrib_or_default c "max" default_max;
      averaged = ExtXml.attrib_or_default c "average" "0"
    }


let _ =
  if Array.length Sys.argv < 2 then
    failwith "Usage: gen_radio xml_file";
  let xml_file = Sys.argv.(1) in
  let xml = Xml.parse_file xml_file in

  printf "/* This file has been generated from %s */\n" xml_file;
  printf "/* Please DO NOT EDIT */\n\n";
  printf "#ifndef %s\n" h_name;
  define h_name "";
  nl ();
  let channels = Xml.children xml in
  let n = ExtXml.attrib xml "name" in
  Xml2h.warning ("RADIO MODEL: "^n);
  define_string "RADIO_NAME" n;
  nl ();
  define "RADIO_CTL_NB" (string_of_int (List.length channels));
  nl ();
  
  (* For compatibility *)
  define "PPM_PULSE_NEUTRAL_US" default_neutral;
  nl ();
  let channels_params = List.map parse_channel channels in 
  nl ();
  define "RADIO_MINS_US" (sprint_float_array (List.map (fun x -> x.min) channels_params));
  define "RADIO_NEUTRALS_US" (sprint_float_array (List.map (fun x -> x.neutral) channels_params));
  define "RADIO_MAXS_US" (sprint_float_array (List.map (fun x -> x.max) channels_params));
  define "RADIO_AVERAGED" (sprint_float_array (List.map (fun x -> x.averaged) channels_params));
  
  nl ();
  define "AveragedChannel(ch)" "(((int[])RADIO_AVERAGED)[ch])";
  
  printf "\n#endif // %s\n" h_name
	
