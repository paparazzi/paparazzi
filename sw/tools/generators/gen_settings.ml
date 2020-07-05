(*
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
let tab = fun out -> fprintf out "%s" (String.make !margin ' ')
let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun out f -> tab out; fprintf out f

let rec flatten = fun xml r ->
  if ExtXml.tag_is xml "dl_setting" then
    xml::r
  else
    match Xml.children xml with
        [] -> r
      | x::xs ->
        List.iter (fun y -> assert(ExtXml.tag_is y (Xml.tag x))) xs;
        List.fold_right flatten (x::xs) r


let print_dl_settings = fun out settings settings_xml ->
  let settings_xml = flatten settings_xml [] in

  (** include  headers **)
  lprintf out "\n";
  List.iter (fun s ->
    List.iter (fun h ->
      lprintf out "#include \"%s.h\"\n" h
    ) (Settings.get_headers s)
  ) settings;
  lprintf out "#include \"generated/modules.h\"\n";
  lprintf out "\n";

  (** Datalink knowing what settings mean **)
  Xml2h.define_out out "SETTINGS_NAMES" "{ \\";
  List.iter (fun b -> fprintf out " { \"%s\" }, \\\n" (ExtXml.attrib b "var")) settings_xml;
  lprintf out "};\n";

  Xml2h.define_out out "SETTINGS_NAMES_SHORT" "{ \\";
  List.iter (fun b ->
    let varname = Str.split (Str.regexp "[_.]+") (ExtXml.attrib b "var") in
    let shortname = List.fold_left (fun acc c ->
      try acc ^"_"^ (Str.first_chars c 3) with _ -> acc ^"_"^ c
    ) "" varname in
    let shorted = try String.sub shortname 1 16 with _ -> String.sub shortname 1 ((String.length shortname)-1) in
    fprintf out " \"%s\" , \\\n" shorted
  ) settings_xml;
  lprintf out "};\n";
  Xml2h.define_out out "NB_SETTING" (string_of_int (List.length settings_xml));

  (** Macro to call to set one variable *)
  lprintf out "#define DlSetting(_idx, _value) { \\\n";
  right ();
  lprintf out "switch (_idx) { \\\n";
  right ();
  let idx = ref 0 in
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      begin
        try
          let h = ExtXml.attrib s "handler" in
          begin
            let m1 = ExtXml.attrib_or_default s "module" "" in
            let m2 = ExtXml.attrib_or_default s "header" "" in
            match m1, m2 with
            | "", "" -> prerr_endline (sprintf "Error: You need to specify the header (or module for legacy) attribute for setting %s to use the handler %s" v h); exit 1
            | "", _ -> lprintf out "case %d: %s_%s( _value ); _value = %s; break;\\\n" !idx (Filename.basename m2) h v
            | _, "" -> lprintf out "case %d: %s_%s( _value ); _value = %s; break;\\\n" !idx (Filename.basename m1) h v
            | _, _ -> prerr_endline (sprintf "Error: You can't specify both module and header attributes for setting %s to use the handler %s" v h); exit 1
          end;
        with
            ExtXml.Error e -> lprintf out "case %d: %s = _value; break;\\\n" !idx v
      end;
      incr idx
    )
    settings_xml;
  lprintf out "default: break;\\\n";
  left ();
  lprintf out "}\\\n";
  left ();
  lprintf out "}\n";
  let nb_values = !idx in

  (** Macro to call to downlink current values *)
  lprintf out "#define PeriodicSendDlValue(_trans, _dev) { \\\n";
  if nb_values > 0 then begin
    right ();
    lprintf out "static uint8_t i;\\\n";
    lprintf out "float var;\\\n";
    lprintf out "if (i >= %d) i = 0;\\\n" nb_values;
    let idx = ref 0 in
    lprintf out "switch (i) { \\\n";
    right ();
    List.iter
      (fun s ->
        let v = ExtXml.attrib s "var" in
        lprintf out "case %d: var = %s; break;\\\n" !idx v; incr idx)
      settings_xml;
    lprintf out "default: var = 0.; break;\\\n";
    left ();
    lprintf out "}\\\n";
    lprintf out "pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\\\n";
    lprintf out "i++;\\\n";
    left ()
  end;
  lprintf out "}\n";

  (** Inline function to get a setting value *)
  lprintf out "static inline float settings_get_value(uint8_t i) {\n";
  right ();
  let idx = ref 0 in
  lprintf out "switch (i) {\n";
  right ();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf out "case %d: return %s;\n" !idx v; incr idx)
    settings_xml;
  lprintf out "default: return 0.;\n";
  left ();
  lprintf out "}\n";
  left ();
  lprintf out "}\n"


let inttype = function
"bool" -> "uint8_t"
  | "int8" -> "int8_t"
  | "int16" -> "int16_t"
  | "int32" -> "int32_t"
  | "int64" -> "int64_t"
  | "uint8" -> "uint8_t"
  | "uint16" -> "uint16_t"
  | "uint32" -> "uint32_t"
  | "uint64" -> "uint64_t"
  | "float" -> "float"
  | "double" -> "double"
  | x -> failwith (sprintf "Gen_calib.inttype: unknown type '%s'" x)


(*
  Generate code for persistent settings
*)
let print_persistent_settings = fun out settings settings_xml ->
  let settings_xml = flatten settings_xml [] in
  let pers_settings =
    List.filter (fun x -> try let _ = Xml.attrib x "persistent" in true with _ -> false) settings_xml in
  (* structure declaration *)
  (*  if List.length pers_settings > 0 then begin *)
  lprintf out "\n/* Persistent Settings */\n";
  lprintf out "struct PersistentSettings {\n";
  right();
  let idx = ref 0 in
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      let t = try ExtXml.attrib s "type" with _ -> "float" in
      lprintf out "%s s_%d; /* %s */\n" (inttype t) !idx v; incr idx)
    pers_settings;
  left();
  lprintf out "};\n\n";
  lprintf out "extern struct PersistentSettings pers_settings;\n\n";
  (*  Inline function to store persistent settings *)
  idx := 0;
  lprintf out "static inline void persistent_settings_store( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf out "pers_settings.s_%d = %s;\n" !idx v; incr idx)
    pers_settings;
  left();
  lprintf out "}\n\n";
  (*  Inline function to load persistent settings *)
  idx := 0;
  lprintf out "static inline void persistent_settings_load( void ) {\n";
  right();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      begin
        try
          let h = ExtXml.attrib s "handler" and
              m =  ExtXml.attrib s "module" in
          lprintf out "%s_%s( pers_settings.s_%d );\n"  (Filename.basename m) h !idx ;
        (*     lprintf out "%s = pers_settings.s_%d;\n" v !idx *) (* do we want to set the value too or just call the handler ? *)
        with
            ExtXml.Error e ->  lprintf out "%s = pers_settings.s_%d;\n" v !idx
      end;
      incr idx)
    pers_settings;
  left();
  lprintf out "}\n"
(*  end *)

let h_name = "SETTINGS_H"

let generate = fun settings xml_files out_xml out_file ->
  let out = open_out out_file in

  (* generate XML concatenated file *)
  let xml = Settings.get_settings_xml settings in
  let f = open_out out_xml in
  fprintf f "%s\n" (ExtXml.to_string_fmt xml);
  close_out f;

  (* generate C file *)
  begin_out out (String.concat " " xml_files) h_name;
  print_dl_settings out settings xml;
  print_persistent_settings out settings xml;
  finish_out out h_name

