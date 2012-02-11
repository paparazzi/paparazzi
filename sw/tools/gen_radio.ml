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


let norm1_ppm = fun c ->
  if c.neutral = c.min then
    sprintf "tmp_radio * (MAX_PPRZ / (float)(RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d)))" c.max c.min, "0"
  else
    sprintf "tmp_radio * (tmp_radio >=0 ? (MAX_PPRZ/(float)(RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d))) : (MIN_PPRZ/(float)(RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d))))" c.max c.neutral c.min c.neutral, "MIN_PPRZ"

let gen_normalize_ppm_fir = fun channels ->
  printf "#define NormalizePpmFIR(_ppm, _rc) {\\\n";
  printf "  static uint8_t avg_cpt = 0; /* Counter for averaging */\\\n";
  printf "  int16_t tmp_radio;\\\n";
  List.iter
    (fun c ->
      let value, min_pprz = norm1_ppm c in
      if c.averaged then begin
        printf "  _rc.avg_values[RADIO_%s] += _ppm[RADIO_%s];\\\n" c.name c.name
      end else begin
        printf "  tmp_radio = _ppm[RADIO_%s] - RC_PPM_TICKS_OF_USEC(%d);\\\n" c.name c.neutral;
        printf "  _rc.values[RADIO_%s] = %s;\\\n" c.name value;
        printf "  Bound(_rc.values[RADIO_%s], %s, MAX_PPRZ); \\\n\\\n" c.name min_pprz;
      end
    )
    channels;
  printf "  avg_cpt++;\\\n";
  printf "  if (avg_cpt == RC_AVG_PERIOD) {\\\n";
  printf "    avg_cpt = 0;\\\n";
  List.iter
    (fun c ->
      if c.averaged then begin
        let value, min_pprz = norm1_ppm c in
        printf "    tmp_radio = _rc.avg_values[RADIO_%s] / RC_AVG_PERIOD -  RC_PPM_TICKS_OF_USEC(%d);\\\n" c.name c.neutral;
        printf "    _rc.values[RADIO_%s] = %s;\\\n" c.name value;
        printf "    _rc.avg_values[RADIO_%s] = 0;\\\n" c.name;
        printf "    Bound(_rc.values[RADIO_%s], %s, MAX_PPRZ); \\\n\\\n" c.name min_pprz;
      end
    )
    channels;
  printf " }\\\n";
  printf "}\n"

let norm1_ppm2 = fun c ->
  if c.neutral = c.min then
    sprintf "(tmp_radio * MAX_PPRZ) / (RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d))" c.max c.min, "0"
  else
    sprintf "(tmp_radio >=0 ? (tmp_radio *  MAX_PPRZ) / (RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d)) : (tmp_radio * MIN_PPRZ) / (RC_PPM_SIGNED_TICKS_OF_USEC(%d-%d)))" c.max c.neutral c.min c.neutral, "MIN_PPRZ"

let gen_normalize_ppm_iir = fun channels ->
  printf "#define NormalizePpmIIR(_ppm, _rc) {\\\n";
  printf "  int32_t tmp_radio;\\\n";
  printf "  int32_t tmp_value;\\\n\\\n";
  List.iter
    (fun c ->
      let value, min_pprz = norm1_ppm2 c in
      printf "  tmp_radio = _ppm[RADIO_%s] - RC_PPM_TICKS_OF_USEC(%d);\\\n" c.name c.neutral;
      printf "  tmp_value = %s;\\\n" value;
      printf "  Bound(tmp_value, %s, MAX_PPRZ); \\\n" min_pprz;
      if c.averaged then
        printf "  _rc.values[RADIO_%s] = (pprz_t)((RADIO_FILTER * _rc.values[RADIO_%s] + tmp_value) / (RADIO_FILTER + 1));\\\n\\\n" c.name c.name
      else
        printf "  _rc.values[RADIO_%s] = (pprz_t)(tmp_value);\\\n\\\n" c.name
      )
    channels;
  (*printf "  rc_values_contains_avg_channels = TRUE;\\\n";*)
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
  (*define "RADIO_CONTROL_NB_CHANNEL" (string_of_int (List.length channels));*)
  define "RADIO_CTL_NB" (string_of_int (List.length channels));
  nl ();
  define "RADIO_FILTER" "7";
  nl ();

  let channels_params = List.map parse_channel channels in
  nl ();

  List.iter
    (fun c ->
      begin
        printf "#define RADIO_%s_NEUTRAL %d\n" c.name c.neutral;
        printf "#define RADIO_%s_MIN %d\n" c.name c.min;
        printf "#define RADIO_%s_MAX %d\n" c.name c.max;
      end
    )
    channels_params;
  nl();

  let ppm_pulse_type = ExtXml.attrib xml "pulse_type" in
  let ppm_data_min = ExtXml.attrib xml "data_min" in
  let ppm_data_max = ExtXml.attrib xml "data_max" in
  let ppm_sync_min = ExtXml.attrib xml "sync_min" in
  let ppm_sync_max = ExtXml.attrib xml "sync_max" in

  printf "#define PPM_PULSE_TYPE PPM_PULSE_TYPE_%s\n" ppm_pulse_type;
  printf "#define PPM_DATA_MIN_LEN (%sul)\n" ppm_data_min;
  printf "#define PPM_DATA_MAX_LEN (%sul)\n" ppm_data_max;
  printf "#define PPM_SYNC_MIN_LEN (%sul)\n" ppm_sync_min;
  printf "#define PPM_SYNC_MAX_LEN (%sul)\n" ppm_sync_max;
  nl ();

  gen_normalize_ppm_fir channels_params;
  nl ();
  gen_normalize_ppm_iir channels_params;
  nl ();

  printf "\n#endif // %s\n" h_name

