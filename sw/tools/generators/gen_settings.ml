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

  (** Datalink knowing what settings mean **)
  Xml2h.define "SETTINGS_NAMES" "{ \\";
  List.iter (fun b -> printf " { \"%s\" }, \\\n" (ExtXml.attrib b "var")) settings;
  lprintf "};\n";

  Xml2h.define "SETTINGS_NAMES_SHORT" "{ \\";
  List.iter (fun b ->
    let varname = Str.split (Str.regexp "[_.]+") (ExtXml.attrib b "var") in
    let shortname = List.fold_left (fun acc c ->
      try acc ^"_"^ (Str.first_chars c 3) with _ -> acc ^"_"^ c
    ) "" varname in
    let shorted = try String.sub shortname 1 16 with _ -> String.sub shortname 1 ((String.length shortname)-1) in
    printf " \"%s\" , \\\n" shorted
  ) settings;
  lprintf "};\n";
  Xml2h.define "NB_SETTING" (string_of_int (List.length settings));

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
          let h = ExtXml.attrib s "handler" in
          begin
            try
              let m =  ExtXml.attrib s "module" in
              lprintf "case %d: %s_%s( _value ); _value = %s; break;\\\n" !idx (Filename.basename m) h v
            with
                ExtXml.Error e -> prerr_endline (sprintf "Error: You need to specify the module attribute for setting %s to use the handler %s" v h); exit 1
          end;
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
  lprintf "#define PeriodicSendDlValue(_trans, _dev) { \\\n";
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
    lprintf "pprz_msg_send_DL_VALUE(_trans, _dev, AC_ID, &i, &var);\\\n";
    lprintf "i++;\\\n";
    left ()
  end;
  lprintf "}\n";

  (** Inline function to get a setting value *)
  lprintf "static inline float settings_get_value(uint8_t i) {\n";
  right ();
  let idx = ref 0 in
  lprintf "switch (i) {\n";
  right ();
  List.iter
    (fun s ->
      let v = ExtXml.attrib s "var" in
      lprintf "case %d: return %s;\n" !idx v; incr idx)
    settings;
  lprintf "default: return 0.;\n";
  left ();
  lprintf "}\n";
  left ();
  lprintf "}\n"


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
      let t = try ExtXml.attrib s "type" with _ -> "float" in
      lprintf "%s s_%d; /* %s */\n" (inttype t) !idx v; incr idx)
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
        (*     lprintf "%s = pers_settings.s_%d;\n" v !idx *) (* do we want to set the value too or just call the handler ? *)
        with
            ExtXml.Error e ->  lprintf "%s = pers_settings.s_%d;\n" v !idx
      end;
      incr idx)
    pers_settings;
  left();
  lprintf "}\n"
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

let parse_rc_setting = fun xml ->
  let cursor, cm = calib_mode_of_rc (ExtXml.attrib xml "rc")
  and var = ExtXml.attrib xml "var"
  and range = float_of_string (ExtXml.attrib xml "range") in
  let t = (ExtXml.attrib xml "type") in
  let param_macro = param_macro_of_type t in
  let dot_pos =
    try String.rindex var '.' + 1 with
        Not_found -> 0 in
  let var_nostruct = String.sub var dot_pos (String.length var - dot_pos) in
  let var_init = var_nostruct ^ "_init" in

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

(*
 Check if target t is marked as supported in the targets string.
 The targets string is a pipe delimited list of supported targets, e.g. "ap|nps"
 To specifiy a list with unsupported targets, prefix with !
 e.g. "!sim|nps" to mark support for all targets except sim and nps.
*)
let supports_target = fun t targets ->
  if String.length targets > 0 && targets.[0] = '!' then
    not (Str.string_match (Str.regexp (".*"^t^".*")) targets 0)
  else
    Str.string_match (Str.regexp (".*"^t^".*")) targets 0

let join_xml_files = fun xml_files ->
  let dl_settings = ref []
  and rc_settings = ref [] in
  let target = try Sys.getenv "TARGET" with _ -> "" in
  List.iter (fun xml_file ->
    (* look for a specific name after settings file (in case of modules) *)
    let split = Str.split (Str.regexp "~") xml_file in
    let xml_file, name = match split with
    | [f; n] -> f, n
    | _ -> xml_file, ""
    in
    let xml = Xml.parse_file xml_file in
    let these_rc_settings =
      try Xml.children (ExtXml.child xml "rc_settings") with
          Not_found -> [] in
    let these_dl_settings =
      try
        (* test if the file is plain settings file or a module file *)
        let xml =
          if Xml.tag xml = "module"
          then begin
            (* test if the module is loaded or not *)
            if List.exists (fun n ->
              if Xml.tag n = "makefile" then begin
                let t = ExtXml.attrib_or_default n "target" Env.default_module_targets in
                supports_target target t
              end
              else false
              ) (Xml.children xml)
            then List.filter (fun t -> Xml.tag t = "settings") (Xml.children xml)
            else []
          end
          else begin
            (* if the top <settings> node has a target attribute,
               only add if matches current target *)
            let t = ExtXml.attrib_or_default xml "target" "" in
            if t = "" || (supports_target target t) then
              [xml]
            else
              []
          end
        in
        (* include settings if name is matching *)
        List.fold_left (fun l x ->
          if (ExtXml.attrib_or_default x "name" "") = name then
            l @ (Xml.children (ExtXml.child x "dl_settings"))
          else l
        ) [] xml
      with
      | Not_found -> [] in
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
    printf "/* This file has been generated by gen_settings from %s */\n" (String.concat " " !xml_files);
    printf "/* Version %s */\n" (Env.get_paparazzi_version ());
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
