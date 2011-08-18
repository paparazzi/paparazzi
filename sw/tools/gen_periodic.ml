(*
 * $Id$
 *
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

let output_modes = fun avr_h process_name channel_name modes freq modules ->
  let min_period = 1./.float freq in
  let max_period = 65536. /. float freq in
  (** For each mode in this process *)
  List.iter
    (fun mode ->
      let mode_name = ExtXml.attrib mode "name" in
      lprintf avr_h "if (telemetry_mode_%s_%s == TELEMETRY_MODE_%s_%s_%s) {\\\n" process_name channel_name process_name channel_name mode_name;
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
        lprintf avr_h "static %s %s = 0; %s++; if (%s>=%d) %s=0;\\\n" _type v v v m v;
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
          lprintf avr_h "%sif (i%d == %d) {\\\n" else_ p !phase;
          l := (p, !phase) :: !l;
          i := !i + freq/10;
          right ();
          lprintf avr_h "PERIODIC_SEND_%s(%s);\\\n" message_name channel_name;
          left ();
          lprintf avr_h "} \\\n"
        )
        messages;
      left ();
      lprintf avr_h "}\\\n")
    modes


let _ =
  if Array.length Sys.argv <> 5 then begin
    failwith (sprintf "Usage: %s <airframe.xml> <messages.xml> <telemetry.xml> frequency_in_hz" Sys.argv.(0))
  end;

  let freq = int_of_string(Sys.argv.(4)) in
  let telemetry_xml =
    try
      Xml.parse_file Sys.argv.(3)
    with Dtd.Check_error e -> failwith (Dtd.check_error e)

  in
  let modules_name = GC.get_modules_name (ExtXml.parse_file Sys.argv.(1)) in

  let avr_h = stdout in

  fprintf avr_h "/* This file has been generated from %s and %s */\n" Sys.argv.(2) Sys.argv.(3);
  fprintf avr_h "/* Please DO NOT EDIT */\n\n";
  fprintf avr_h "#ifndef _VAR_PERIODIC_H_\n";
  fprintf avr_h "#define _VAR_PERIODIC_H_\n";

  (** For each process *)
  List.iter
    (fun process ->
      let process_name = ExtXml.attrib process "name" and
      channel_name =  ExtXml.attrib process "channel" in

      fprintf avr_h "\n/* Macros for %s process channel %s */\n" process_name channel_name;

      let modes = Xml.children process in

      let i = ref 0 in
      (** For each mode of this process *)
      List.iter (fun mode ->
        let name = ExtXml.attrib mode "name" in
        Xml2h.define (sprintf "TELEMETRY_MODE_%s_%s_%s" process_name channel_name name) (string_of_int !i);
        (* Output the periods of the messages *)
        List.iter
          (fun x ->
            let p = ExtXml.attrib x "period"
            and n = ExtXml.attrib x "name" in
            Xml2h.define (sprintf "PERIOD_%s_%s_%s_%d" n process_name channel_name !i) (sprintf "(%s)" p))
          (Xml.children mode);
        incr i)
        modes;

      lprintf avr_h "#define PeriodicSend%s(%s) {  /* %dHz */ \\\n" process_name channel_name freq;
      right ();
      output_modes avr_h process_name channel_name modes freq modules_name;
      left ();
      lprintf avr_h "}\n"
    )
    (Xml.children telemetry_xml);

  fprintf avr_h "#endif // _VAR_PERIODIC_H_\n";

