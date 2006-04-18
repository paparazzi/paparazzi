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

let margin = ref 0
let step = 2

let right () = margin := !margin + step
let left () = margin := !margin - step

let lprintf = fun c f ->
  fprintf c "%s" (String.make !margin ' ');
  fprintf c f

let freq = 10
let nb_steps = 100

let remove_dup = fun l ->
  let rec loop = fun l ->
    match l with
      [] | [_] -> l
    | x::((x'::_) as xs) ->
	if x = x' then loop xs else x::loop xs in
  loop (List.sort compare l)

let _ =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <messages.xml> <telemetry.xml>" Sys.argv.(0)) 
  end;

  let messages_xml = Xml.parse_file Sys.argv.(1) in
  let telemetry_xml = 
    try
      Xml.parse_file Sys.argv.(2)
    with Dtd.Check_error e -> failwith (Dtd.check_error e)
      
  in

  let avr_h = stdout in

  fprintf avr_h "/* This file has been generated from %s and %s */\n" Sys.argv.(1) Sys.argv.(2);
  fprintf avr_h "/* Please DO NOT EDIT */\n\n";
  
  fprintf avr_h "/* Periodic messages are sent over %d timeframes */\n" nb_steps;

  (** For each process *)
  List.iter
    (fun process ->
      let process_name = ExtXml.attrib process "name" in

      fprintf avr_h "\n/* Macros for %s process */\n" process_name;

      let modes = Xml.children process in
      
      let i = ref 0 in
      List.iter (fun mode ->
	let name = ExtXml.attrib mode "name" in
	Xml2h.define (sprintf "TELEMETRY_MODE_%s_%s" process_name name) (string_of_int !i);
	incr i)
	modes;

      lprintf avr_h "#define PeriodicSend%s() {  /* %dHz */ \\\n" process_name freq;
      right ();
      lprintf avr_h "static uint8_t periodic_i;\\\n";
      lprintf avr_h "periodic_i++; if (periodic_i == %d) periodic_i = 0;\\\n" nb_steps;
      
      (** For each mode in this process *)
      List.iter
	(fun mode ->
	  let mode_name = ExtXml.attrib mode "name" in
	  lprintf avr_h "if (telemetry_mode_%s == TELEMETRY_MODE_%s_%s) {\\\n" process_name process_name mode_name;
	  right ();

	  (** Computes the required modulos *)
	  let messages =
	    List.map
	      (fun x -> 
		let p = float_of_string (ExtXml.attrib x "period") in
		x, int_of_float (p*.float_of_int freq))
	      (Xml.children mode) in
	  let modulos = remove_dup (List.map snd messages) in
	  List.iter
	    (fun m ->
	      lprintf avr_h "uint8_t i_mod_%d = periodic_i %% %d; \\\n" m m;
	    )
	    modulos;
	  
	  
	  (** For each message in this mode *)
	  List.iter
	    (fun (message, p) ->
	      assert(p < nb_steps);
	      let message_name = ExtXml.attrib message "name" in
	      lprintf avr_h "if (i_mod_%d == 0) {\\\n" p;
	      right ();
	      lprintf avr_h "PERIODIC_SEND_%s();\\\n" message_name;
	      left ();
	      lprintf avr_h "}\\\n")
	    messages;
	  left ();
	  lprintf avr_h "}\\\n")
	modes;
      left ();
      lprintf avr_h "}\n"
    )
    (Xml.children telemetry_xml)

