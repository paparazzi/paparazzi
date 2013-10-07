(*
 * XML preprocessing for periodic messages
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
module GC = Gen_common

let (//) = Filename.concat

let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun c f ->
  fprintf c "%s" (String.make !margin ' ');
  fprintf c f

let output_modes = fun out_h process_name modes freq modules ->
  let min_period = 1./.float freq in
  let max_period = 65536. /. float freq in
  (** For each mode in this process *)
  List.iter
    (fun mode ->
      let mode_name = ExtXml.attrib mode "name" in
      lprintf out_h "if (telemetry_mode_%s == TELEMETRY_MODE_%s_%s) {\\\n" process_name process_name mode_name;
      right ();

      (** Filter message list to remove messages linked to unloaded modules *)
      let filtered_msg = List.filter (fun msg ->
        try let att = Xml.attrib msg "module" in List.exists (fun name -> String.compare name att = 0) modules with _ -> true
      ) (Xml.children mode) in
      (** Computes the required modulos *)
      let messages = List.map (fun x ->
        let p = float_of_string (ExtXml.attrib x "period") in
        if p < min_period || p > max_period then
          fprintf stderr "Warning: period is bound between %.3fs and %.3fs for message %s\n%!" min_period max_period (ExtXml.attrib x "name");
        (x, min 65535 (max 1 (int_of_float (p*.float_of_int freq))))
      ) filtered_msg in
      let modulos = GC.singletonize (List.map snd messages) in
      List.iter (fun m ->
        let v = sprintf "i%d" m in
        let _type = if m >= 256 then "uint16_t" else "uint8_t" in
        lprintf out_h "static %s %s = 0; %s++; if (%s>=%d) %s=0;\\\n" _type v v v m v;
      ) modulos;

      (** For each message in this mode *)
      let messages = List.sort (fun (_,p) (_,p') -> compare p p') messages in
      let i = ref 0 in (** Basic balancing:1 message every 10Hz *)
      let phase = ref 0 in
      let l = ref [] in
      List.iter
        (fun (message, p) ->
          let message_name = ExtXml.attrib message "name" in
          i := !i mod p;
          (* if phase attribute is present, use it, otherwise shedule at 10Hz *)
          let message_phase = try int_of_float (float_of_string (ExtXml.attrib message "phase")*.float_of_int freq) with _ -> !i in
          phase := message_phase;
          let else_ = if List.mem_assoc p !l && not (List.mem (p, !phase) !l) then "else " else "" in
          lprintf out_h "%sif (i%d == %d) {\\\n" else_ p !phase;
          l := (p, !phase) :: !l;
          i := !i + freq/10;
          right ();
          lprintf out_h "PERIODIC_SEND_%s(_trans, _dev);\\\n" message_name;
          left ();
          lprintf out_h "} \\\n"
        )
        messages;
      left ();
      lprintf out_h "}\\\n")
    modes

let write_settings = fun xml_file out_set telemetry_xml ->
  fprintf out_set "<!-- This file has been generated from %s -->\n" xml_file;
  fprintf out_set "<!-- Please DO NOT EDIT -->\n\n";
  fprintf out_set "<settings>\n";
  fprintf out_set " <dl_settings>\n";
  fprintf out_set "  <dl_settings name=\"Telemetry\">\n";
  List.iter (fun p ->
    (* for each process *)
    let process_name = Xml.attrib p "name" in
    (* convert the xml list of mode to a string list *)
    let modes = List.map (fun m -> Xml.attrib m "name") (Xml.children p) in
    let nb_modes = List.length modes in
    match nb_modes with
        0 | 1 -> () (* Nothing to do if 1 or zero mode *)
      | _ -> (* add settings with all modes *)
        fprintf out_set "   <dl_setting min=\"0\" step=\"1\" max=\"%d\" var=\"telemetry_mode_%s\" shortname=\"%s\" values=\"%s\">\n" (nb_modes-1) process_name process_name (String.concat "|" modes);
        let i = ref 0 in
        List.iter (fun m -> try
                              let key = Xml.attrib m "key_press" in
                              fprintf out_set "    <key_press key=%S value=%S/>\n" key (string_of_int !i);
                              incr i
          with _ -> incr i) (Xml.children p);
        fprintf out_set "   </dl_setting>\n"
  ) (Xml.children telemetry_xml);
  fprintf out_set "  </dl_settings>\n";
  fprintf out_set " </dl_settings>\n";
  fprintf out_set "</settings>\n"


let _ =
  if Array.length Sys.argv <> 6 then begin
    failwith (sprintf "Usage: %s <airframe.xml> <messages.xml> <telemetry.xml> frequency_in_hz out_settings_file" Sys.argv.(0))
  end;

  let freq = int_of_string(Sys.argv.(4)) in
  let telemetry_xml =
    try
      Xml.parse_file Sys.argv.(3)
    with Dtd.Check_error e -> failwith (Dtd.check_error e)

  in
  let modules_name = GC.get_modules_name (ExtXml.parse_file Sys.argv.(1)) in

  let out_h = stdout in

  fprintf out_h "/* This file has been generated from %s and %s */\n" Sys.argv.(2) Sys.argv.(3);
  fprintf out_h "/* Please DO NOT EDIT */\n\n";
  fprintf out_h "#ifndef _VAR_PERIODIC_H_\n";
  fprintf out_h "#define _VAR_PERIODIC_H_\n\n";
  fprintf out_h "#include \"std.h\"\n";
  fprintf out_h "#include \"generated/airframe.h\"\n\n";
  fprintf out_h "#define TELEMETRY_FREQUENCY %d\n\n" freq;

  (** For each process *)
  List.iter
    (fun process ->
      let process_name = ExtXml.attrib process "name" in

      fprintf out_h "\n/* Macros for %s process */\n" process_name;
      fprintf out_h "#ifdef PERIODIC_C_%s\n" (String.uppercase process_name);
      fprintf out_h "#ifndef TELEMETRY_MODE_%s\n" (String.uppercase process_name);
      fprintf out_h "#define TELEMETRY_MODE_%s 0\n" (String.uppercase process_name);
      fprintf out_h "#endif\n";
      fprintf out_h "uint8_t telemetry_mode_%s = TELEMETRY_MODE_%s;\n" process_name (String.uppercase process_name);
      fprintf out_h "#else /* PERIODIC_C_%s not defined (general header) */\n" (String.uppercase process_name);
      fprintf out_h "extern uint8_t telemetry_mode_%s;\n" process_name;
      fprintf out_h "#endif /* PERIODIC_C_%s */\n" (String.uppercase process_name);

      let modes = Xml.children process in

      let i = ref 0 in
      (** For each mode of this process *)
      List.iter (fun mode ->
        let name = ExtXml.attrib mode "name" in
        Xml2h.define (sprintf "TELEMETRY_MODE_%s_%s" process_name name) (string_of_int !i);
        (* Output the periods of the messages *)
        List.iter
          (fun x ->
            let p = ExtXml.attrib x "period"
            and n = ExtXml.attrib x "name" in
            Xml2h.define (sprintf "PERIOD_%s_%s_%d" n process_name !i) (sprintf "(%s)" p))
          (Xml.children mode);
        incr i)
        modes;

      lprintf out_h "#define PeriodicSend%s(_trans, _dev) {  /* %dHz */ \\\n" process_name freq;
      right ();
      output_modes out_h process_name modes freq modules_name;
      left ();
      lprintf out_h "}\n"
    )
    (Xml.children telemetry_xml);

  (** Output XML settings file with telemetry modes *)
  let out_set = open_out Sys.argv.(5) in
  write_settings Sys.argv.(3) out_set telemetry_xml;
  close_out out_set;

  fprintf out_h "#endif // _VAR_PERIODIC_H_\n";

