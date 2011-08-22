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


module StringSet = Set.Make(struct type t = string let compare = compare end)


let print_dl_settings = fun settings ->
  let settings = flatten settings [] in

  (** include  headers **)
  let modules = ref StringSet.empty in
  List.iter
    (fun s ->
      try
    modules := StringSet.add (ExtXml.attrib s "module") !modules
      with ExtXml.Error e -> ()
    )
    settings;

  lprintf "\n";
  StringSet.iter (fun m -> lprintf "#include \"%s.h\"\n" m) !modules;
  lprintf "#include \"generated/modules.h\"\n";
  lprintf "\n";

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
      lprintf "case %d: %s_%s( _value ); _value = %s; break;\\\n" !idx (Filename.basename m) h v
    with
      ExtXml.Error e -> lprintf "case %d: %s = _value; break;\\\n" !idx v
    end;
    incr idx
    )
    settings;
  lprintf "default: break;\\\n";
  left ();
  lprintf "}\\\n";
  left ();
  lprintf "}\n";
  let nb_values = !idx in

  (** Macro to call to downlink current values *)
  lprintf "#define PeriodicSendDlValue(_chan) { \\\n";
  if nb_values > 0 then begin
    right ();
    lprintf "static uint8_t i;\\\n";
    lprintf "float var;\\\n";
    lprintf "if (i >= %d) i = 0;\\\n" nb_values;
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
    lprintf "DOWNLINK_SEND_DL_VALUE(_chan, &i, &var);\\\n";
    lprintf "i++;\\\n";
    left ()
  end;
  lprintf "}\n";

  (** Inline function to get a setting value *)
  lprintf "static inline float settings_get_value(uint8_t i) {\n";
  right ();
  let idx = ref 0 in
  lprintf "switch (i) { \\\n";
  right ();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf "case %d: return %s;\n" !idx v; incr idx)
    settings;
  lprintf "default: return 0.;\n";
  lprintf "}\n";
  left ();
  lprintf "}\n";
  left()

(*
   Generate code for persistent settings
*)
let print_persistent_settings = fun settings ->
  let settings = flatten settings [] in
  let pers_settings =
    List.filter (fun x -> try let _ = Xml.attrib x "persistent" in true with _ -> false) settings in
  (* structure declaration *)
(*  if List.length pers_settings > 0 then begin *)
  lprintf "\n/* Persistent Settings */\n";
  lprintf "struct PersistentSettings {\n";
  right();
  let idx = ref 0 in
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf "float s_%d; /* %s */\n" !idx v; incr idx)
    pers_settings;
  left();
  lprintf "};\n\n";
  lprintf "extern struct PersistentSettings pers_settings;\n\n";
  (*  Inline function to store persistent settings *)
  idx := 0;
  lprintf "static inline void persistent_settings_store( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf "pers_settings.s_%d = %s;\n" !idx v; incr idx)
    pers_settings;
  left();
  lprintf "}\n\n";
  (*  Inline function to load persistent settings *)
  idx := 0;
  lprintf "static inline void persistent_settings_load( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      begin
	 try
	   let h = ExtXml.attrib s "handler" and
               m =  ExtXml.attrib s "module" in
	   lprintf "%s_%s( pers_settings.s_%d );\n"  (Filename.basename m) h !idx ;
(*	   lprintf "%s = pers_settings.s_%d;\n" v !idx *) (* do we want to set the value too or just call the handler ? *)
	 with
	   ExtXml.Error e ->  lprintf "%s = pers_settings.s_%d;\n" v !idx
      end;
      incr idx)
    pers_settings;
  left();
  lprintf "};\n"
(*  end *)


(*
   Blaaaaaa2
*)
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


let join_xml_files = fun xml_files ->
  let dl_settings = ref []
  and rc_settings = ref [] in
  List.iter (fun xml_file ->
    let xml = Xml.parse_file xml_file in
    let these_rc_settings =
      try Xml.children (ExtXml.child xml "rc_settings") with
    Not_found -> [] in
    let these_dl_settings =
      try Xml.children (ExtXml.child xml "dl_settings") with
    Not_found -> [] in
    rc_settings := these_rc_settings @ !rc_settings;
    dl_settings := these_dl_settings @ !dl_settings)
    xml_files;
  Xml.Element("rc_settings",[],!rc_settings), Xml.Element("dl_settings",[],!dl_settings)



let _ =
  if Array.length Sys.argv < 4 then
    failwith (Printf.sprintf "Usage: %s output_xml_file input_xml_file(s) input_xml_modules" Sys.argv.(0));
  let h_name = "SETTINGS_H"
  and xml_files = ref [] in
  for i = 2 to Array.length Sys.argv - 1 do
    xml_files := Sys.argv.(i) :: !xml_files;
  done;

  try
    printf "/* This file has been generated from %s */\n" (String.concat " " !xml_files);
    printf "/* Please DO NOT EDIT */\n\n";

    printf "#ifndef %s\n" h_name;
    define h_name "";
    nl ();

    let rc_settings, dl_settings = join_xml_files !xml_files in

    let xml = Xml.Element ("settings", [], [rc_settings; dl_settings]) in
    let f = open_out Sys.argv.(1) in
    fprintf f "%s\n" (ExtXml.to_string_fmt xml);
    close_out f;

    lprintf "#define RCSettings(mode_changed) { \\\n";
    right ();
    parse_rc_modes rc_settings;
    left (); lprintf "}\n";

    print_dl_settings dl_settings;

    print_persistent_settings dl_settings;

    finish h_name
  with
    Xml.Error e -> prerr_endline (Xml.error e); exit 1
  | Dtd.Prove_error e ->  prerr_endline (Dtd.prove_error e); exit 1
  | Dtd.Parse_error e ->  prerr_endline (Dtd.parse_error e); exit 1
