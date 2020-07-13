(*
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2017 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                    Cyril Allignol <cyril.allignol@enac.fr>
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *)

(**
 * XML preprocessing for periodic messages
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

let output_modes = fun out_h process_name telem_type modes ->
  (** For each mode in this process *)
  List.iter
    (fun mode ->
      let mode_name = ExtXml.attrib mode "name" in
      lprintf out_h "if (telemetry_mode_%s == TELEMETRY_MODE_%s_%s) {\n" process_name process_name mode_name;
      right ();

      (** Computes the required modulos *)
      let found_modulos = Hashtbl.create 15 in
      let idx = ref 0 in
      let phase = ref 0. in (** Basic balancing: increment with a step of 0.1 *)
      let messages = List.map (fun x ->
        let period = ExtXml.attrib x "period" in
        let _phase = begin try
            let _p = float_of_string (ExtXml.attrib x "phase") in
            if _p > 0.95 then _p /. 65536. else _p  (* try to keep some backward compatibility *)
          with _ ->
            phase := !phase +. 0.1;
            if !phase > 0.9 then phase := 0.;
            !phase
          end
        in
        try
          ((x, _phase), (period, Hashtbl.find found_modulos period))
        with Not_found ->
          incr idx; (* create new module *)
          let v = sprintf "i%d" !idx in
          lprintf out_h "static uint32_t %s = 0; %s++; if (%s>= (uint32_t)(TELEMETRY_FREQUENCY*%s)) %s=0;\n" v v v period v;
          Hashtbl.add found_modulos period !idx;
          ((x, _phase), (period, !idx))
      ) (Xml.children mode) in

      (* create var to loop trough callbacks if needed *)
      if (List.length messages > 0) then
        lprintf out_h "uint8_t j;\n";

      (** For each message in this mode *)
      let messages = List.sort (fun (_,(p,_)) (_,(p',_)) -> compare p p') messages in
      List.iter
        (fun ((message, _phase), (p, i)) ->
          let message_name = ExtXml.attrib message "name" in
          lprintf out_h "if (i%d == (uint32_t)(TELEMETRY_FREQUENCY*%s*%f)) {\n" i p _phase;
          right ();
          lprintf out_h "for (j = 0; j < TELEMETRY_NB_CBS; j++) {\n";
          right ();
          lprintf out_h "if (telemetry->cbs[TELEMETRY_%s_MSG_%s_IDX].slots[j] != NULL)\n" telem_type message_name;
          right ();
          lprintf out_h "telemetry->cbs[TELEMETRY_%s_MSG_%s_IDX].slots[j](trans, dev);\n" telem_type message_name;
          left ();
          lprintf out_h "else break;\n";
          left ();
          lprintf out_h "}\n";
          fprintf out_h "#if USE_PERIODIC_TELEMETRY_REPORT\n";
          lprintf out_h "if (j == 0) periodic_telemetry_err_report(TELEMETRY_PROCESS_%s, telemetry_mode_%s, %s_MSG_ID_%s);\n" process_name process_name telem_type message_name;
          fprintf out_h "#endif\n";
          left ();
          lprintf out_h "}\n"
        )
        (List.rev messages);
      left ();
      lprintf out_h "}\n")
    modes

let print_message_table = fun out_h xml ->
  let telemetry_types = Hashtbl.create 2 in
  (* For each process *)
  List.iter (fun process ->
    let telem_type = Compat.uppercase_ascii (ExtXml.attrib_or_default process "type" "pprz") in
    if not (Hashtbl.mem telemetry_types telem_type) then Hashtbl.add telemetry_types telem_type (Hashtbl.create 15);
    let messages = Hashtbl.find telemetry_types telem_type in
    (** For each mode of this process *)
    List.iter (fun mode ->
      (** For each message in this mode *)
      List.iter (fun msg ->
        let n = ExtXml.attrib msg "name" in
        (* Add message to the list if it doesn't exist *)
        if not (Hashtbl.mem messages n) then Hashtbl.add messages n ()
      ) (Xml.children mode)
    ) (Xml.children process)
  ) (Xml.children xml);
  (* for each telemetry type, print ID and other defines *)
  Hashtbl.iter (fun telem_type messages ->
    (* Print ID *)
    fprintf out_h "/* Periodic telemetry messages of type %s */\n" telem_type;
    let nb = Hashtbl.fold (fun n _ i ->
      Xml2h.define_out out_h (sprintf "TELEMETRY_%s_MSG_%s_IDX" telem_type n) (sprintf "%d" i);
      i+1
    ) messages 0 in
    fprintf out_h "#define TELEMETRY_%s_NB_MSG %d\n" telem_type nb;
    (* Structure initialization *)
    fprintf out_h "\n#define TELEMETRY_%s_MSG_NAMES { \\\n" telem_type;
    Hashtbl.iter (fun n _ -> fprintf out_h "  \"%s\", \\\n" n) messages;
    fprintf out_h "}\n\n";
    fprintf out_h "#define TELEMETRY_%s_CBS { \\\n" telem_type;
    Hashtbl.iter (fun n _ -> fprintf out_h "  {.id=%s_MSG_ID_%s, .slots={ NULL }}, \\\n" telem_type n) messages;
    fprintf out_h "}\n\n"
  ) telemetry_types

let print_process_send = fun out_h xml ->
  (** For each process *)
  List.iter
    (fun process ->
      let process_name = ExtXml.attrib process "name" in
      let telem_type = Compat.uppercase_ascii (ExtXml.attrib_or_default process "type" "pprz") in
      let modes = Xml.children process in

      fprintf out_h "\n/* Periodic telemetry (type %s): %s process */\n" telem_type process_name;
      let p_id = ref 0 in
      Xml2h.define_out out_h (sprintf "TELEMETRY_PROCESS_%s" process_name) (string_of_int !p_id);
      incr p_id;

      (** For each mode of this process *)
      let _ = List.fold_left (fun i mode ->
        let name = ExtXml.attrib mode "name" in
        Xml2h.define_out out_h (sprintf "TELEMETRY_MODE_%s_%s" process_name name) (string_of_int i);
        (* Output the periods of the messages *)
        List.iter (fun x ->
          let p = ExtXml.attrib x "period"
          and n = ExtXml.attrib x "name" in
          (* FIXME really needed ? *)
          Xml2h.define_out out_h (sprintf "PERIOD_%s_%s_%d" n process_name i) (sprintf "(%s)" p)
        ) (Xml.children mode);
        i + 1
      ) 0 modes in

      fprintf out_h "\n/* Functions for %s process */\n" process_name;
      fprintf out_h "#ifdef PERIODIC_C_%s\n" (Compat.uppercase_ascii process_name);
      fprintf out_h "#ifndef TELEMETRY_MODE_%s\n" (Compat.uppercase_ascii process_name);
      fprintf out_h "#define TELEMETRY_MODE_%s 0\n" (Compat.uppercase_ascii process_name);
      fprintf out_h "#endif\n";
      fprintf out_h "uint8_t telemetry_mode_%s = TELEMETRY_MODE_%s;\n" process_name (Compat.uppercase_ascii process_name);
      fprintf out_h "#else /* PERIODIC_C_%s not defined (general header) */\n" (Compat.uppercase_ascii process_name);
      fprintf out_h "extern uint8_t telemetry_mode_%s;\n" process_name;
      fprintf out_h "#endif /* PERIODIC_C_%s */\n" (Compat.uppercase_ascii process_name);

      lprintf out_h "static inline void periodic_telemetry_send_%s(struct periodic_telemetry *telemetry, struct transport_tx *trans, struct link_device *dev) {\n" process_name;
      right ();
      output_modes out_h process_name telem_type modes;
      left ();
      lprintf out_h "}\n"
    )
    (Xml.children xml)


(**
 * main generation function
 *)
let generate = fun telemetry out_file ->
  let out = open_out out_file in

  (** Print header *)
  fprintf out "/* This file has been generated by gen_periodic */\n";
  fprintf out "/* Version %s */\n" (Env.get_paparazzi_version ());
  fprintf out "/* Please DO NOT EDIT */\n\n";
  fprintf out "#ifndef _VAR_PERIODIC_H_\n";
  fprintf out "#define _VAR_PERIODIC_H_\n\n";
  fprintf out "#include \"std.h\"\n";
  fprintf out "#include \"generated/airframe.h\"\n";
  fprintf out "#include \"subsystems/datalink/telemetry_common.h\"\n";
  fprintf out "\n";
  fprintf out "#ifndef TELEMETRY_FREQUENCY\n";
  fprintf out "#ifdef PERIODIC_FREQUENCY\n";
  fprintf out "#define TELEMETRY_FREQUENCY PERIODIC_FREQUENCY\n";
  fprintf out "#else\n";
  fprintf out "#error \"neither TELEMETRY_FREQUENCY or PERIODIC_FREQUENCY are defined\"\n";
  fprintf out "#endif\n";
  fprintf out "#endif\n";
  fprintf out "\n";

  (** Print the telemetry table with ID *)
  print_message_table out telemetry.Telemetry.xml;

  (** Print process sending functions *)
  print_process_send out telemetry.Telemetry.xml;

  fprintf out "#endif // _VAR_PERIODIC_H_\n";

  close_out out

