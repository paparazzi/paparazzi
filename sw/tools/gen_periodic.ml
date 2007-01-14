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

let freq = 60
let min_period = 1./.float freq
let max_period = 65536. /. float freq

let remove_dup = fun l ->
  let rec loop = fun l ->
    match l with
      [] | [_] -> l
    | x::((x'::_) as xs) ->
	if x = x' then loop xs else x::loop xs in
  loop (List.sort compare l)


let output_modes = fun avr_h process_name modes ->       
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
	    if p < min_period || p > max_period then
	      fprintf stderr "Warning: period is bound between %.3fs and %.3fs for message %s\n%!" min_period max_period (ExtXml.attrib x "name");
	    (x, min 65535 (max 1 (int_of_float (p*.float_of_int freq)))))
	  (Xml.children mode)
      in
      let modulos = remove_dup (List.map snd messages) in
      List.iter
	(fun m ->
	  let v = sprintf "i%d" m in
	  let _type = if m >= 256 then "uint16_t" else "uint8_t" in
	  lprintf avr_h "static %s %s; %s++; if (%s>=%d) %s=0;\\\n" _type v v v m v;
	)
	modulos;
      
      (** For each message in this mode *)
      let messages = List.sort (fun (_,p) (_,p') -> compare p p') messages in
      let i = ref 0 in (** Basic balancing:1 message every 10Hz *)
      let last_p = ref 0 in
      List.iter
	(fun (message, p) ->
	  let message_name = ExtXml.attrib message "name" in
	  i := !i mod p;
	  let else_ = if p == !last_p then "else " else "" in
	  lprintf avr_h "%sif (i%d == %d) {\\\n" else_ p !i;
	  i := !i + freq/10;
	  right ();
	  lprintf avr_h "PERIODIC_SEND_%s();\\\n" message_name;
	  left ();
	  lprintf avr_h "} \\\n";
	  last_p := p
	)
	messages;
      left ();
      lprintf avr_h "}\\\n")
    modes


let _ =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <messages.xml> <telemetry.xml>" Sys.argv.(0)) 
  end;

  let telemetry_xml = 
    try
      Xml.parse_file Sys.argv.(2)
    with Dtd.Check_error e -> failwith (Dtd.check_error e)
      
  in

  let avr_h = stdout in

  fprintf avr_h "/* This file has been generated from %s and %s */\n" Sys.argv.(1) Sys.argv.(2);
  fprintf avr_h "/* Please DO NOT EDIT */\n\n";
  
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
      output_modes avr_h process_name modes;
      left ();
      lprintf avr_h "}\n"
    )
    (Xml.children telemetry_xml)

