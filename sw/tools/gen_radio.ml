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
type us = int

type channel = { 
    name : string;
    min : us; 
    max : us; 
    neutral : us; 
    averaged : bool }


(* Characters used in Gen_airframe.pprz_value *)
let check_function_name = fun s ->
  for i = 0 to String.length s - 1 do
    match s.[i] with
      'A'..'Z' | '0'..'9' | '_' -> ()
    | _ ->
	failwith (sprintf "Character '%c' not allowed in function name '%s'" s.[i] s)
  done

let parse_channel =
  let no_channel = ref 0 in
  fun c ->
    let name = ExtXml.attrib c "function"  in
    check_function_name name;
    let ctl = "RADIO_CTL_"^ExtXml.attrib c "ctl"
    and fct = "RADIO_" ^ name in
    define ctl (string_of_int !no_channel);
    define fct ctl;
    no_channel := !no_channel + 1;
    let int_attrib = fun x -> int_of_string (ExtXml.attrib c x) in
    { min = int_attrib "min";
      neutral = int_attrib "neutral";
      max = int_attrib "max";
      averaged = ExtXml.attrib_or_default c "average" "0" <> "0";
      name = name
    }


let gen_normalize_ppm = fun channels ->
  printf "#define NormalizePpm() {\\\n";
  printf "  static uint8_t avg_cpt = 0; /* Counter for averaging */\\\n";
  printf "   int16_t tmp_radio;\\\n";
  List.iter
    (fun c ->
      printf "  tmp_radio = ppm_pulses[RADIO_%s] -  SYS_TICS_OF_USEC(%d);\\\n" c.name c.neutral;
      let period = if c.averaged then "RC_AVG_PERIOD" else "1" in
      let value, min_pprz = 
	if c.neutral = c.min then
	  sprintf "tmp_radio * (MAX_PPRZ / %s / (float)(SIGNED_SYS_TICS_OF_USEC(%d-%d)))" period c.max c.min, "0"
	else
	  sprintf "tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/%s/(float)(SIGNED_SYS_TICS_OF_USEC(%d-%d))) : (MIN_PPRZ/%s/(float)(SIGNED_SYS_TICS_OF_USEC(%d-%d))))" period c.max c.neutral period c.min c.neutral, "MIN_PPRZ" in
      if c.averaged then begin
	printf "  avg_rc_values[RADIO_%s] += %s;\\\n" c.name value
      end else begin
	printf "  rc_values[RADIO_%s] = %s;\\\n" c.name value;
	printf "  if (rc_values[RADIO_%s] > MAX_PPRZ) rc_values[RADIO_%s] = MAX_PPRZ;\\\n else if (rc_values[RADIO_%s] < %s) rc_values[RADIO_%s] = %s; \\\n\\\n" c.name c.name c.name min_pprz c.name min_pprz;
      end
      )
    channels;
  printf "avg_cpt++;\\\n";
  printf "  if (avg_cpt == RC_AVG_PERIOD) {\\\n";
  printf "    avg_cpt = 0;\\\n";
  List.iter
    (fun c ->
      if c.averaged then begin
	printf "    rc_values[RADIO_%s] = avg_rc_values[RADIO_%s];\\\n" c.name c.name;
	printf "    avg_rc_values[RADIO_%s] = 0;\\\n" c.name;
	printf "  if (rc_values[RADIO_%s] > MAX_PPRZ) rc_values[RADIO_%s] = MAX_PPRZ;\\\n else if (rc_values[RADIO_%s] < MIN_PPRZ) rc_values[RADIO_%s] = MIN_PPRZ; \\\n\\\n" c.name c.name c.name c.name;
      end
    )
    channels;
  printf "    rc_values_contains_avg_channels = TRUE;\\\n";
  printf " }\\\n";
  printf "}\n"



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
  
  let channels_params = List.map parse_channel channels in 
  nl ();
  
  let ppm_data_min = ExtXml.attrib xml "data_min" in
  let ppm_data_max = ExtXml.attrib xml "data_max" in
  let ppm_sync_min = ExtXml.attrib xml "sync_min" in
  let ppm_sync_max = ExtXml.attrib xml "sync_max" in

  printf "#define PPM_DATA_MIN_LEN (%sul)\n" ppm_data_min;
  printf "#define PPM_DATA_MAX_LEN (%sul)\n" ppm_data_max;
  printf "#define PPM_SYNC_MIN_LEN (%sul)\n" ppm_sync_min;
  printf "#define PPM_SYNC_MAX_LEN (%sul)\n" ppm_sync_max;
  nl ();

  gen_normalize_ppm channels_params;
  
  printf "\n#endif // %s\n" h_name
	
