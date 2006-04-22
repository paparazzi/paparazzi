(*
 *  $Id$
 *
 * Hardware In The Loop
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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


open Stdlib
open Latlong

let get_port = fun n ->
  Xml.attrib (ExtXml.child ~select:(fun x -> Xml.attrib x "name"=n) Data.ground "link") "port"

let tty0 = ref (get_port "ap")
let tty1 = ref (get_port "fbw")

let uart_mcu0 = ref Unix.stdout
let uart_mcu1 = ref Unix.stdin

let open_mcu tty = Serial.opendev tty Serial.B38400

module Make(A:Data.MISSION) = struct

  let init = fun (_:int) (_:GPack.box) ->
    if !tty0 <> "" then uart_mcu0 := open_mcu !tty0;
    if !tty1 <> "" then uart_mcu1 := open_mcu !tty1

  let boot = fun time_scale -> ()

    let irs =
      try
	ExtXml.child A.ac.Data.airframe
	  ~select:(fun x -> try Xml.attrib x "prefix" = "IR_" with Xml.No_attribute _ -> false)
	  "section"
      with Not_found -> 
	failwith "Do not find an IR section in airframe description"

    let ir_roll_neutral =
      try
	int_of_string (ExtXml.attrib (ExtXml.child irs ~select:(fun x -> try Xml.attrib x "name" = "ADC_ROLL_NEUTRAL" with Xml.No_attribute _ -> false) "define") "value")
      with
      Not_found ->
	failwith "Do not find an ROLL_NEUTRAL_DEFAULT define in IR description" 
    
    let ir_pitch_neutral =
      try
	int_of_string (ExtXml.attrib (ExtXml.child irs ~select:(fun x -> try Xml.attrib x "name" = "ADC_PITCH_NEUTRAL" with Xml.No_attribute _ -> false) "define") "value")
      with
	Not_found ->
	  failwith "Do not find an PITCH_NEUTRAL_DEFAULT define in IR description"
	    



  let scale = fun value s -> truncate (value *. s)

  open Gps

  let nav_posutm = Ubx.nav_posutm ()
  let nav_status = Ubx.nav_status ()
  let nav_velned = Ubx.nav_velned ()
  let usr_irsim = Ubx.usr_irsim ()

  let gps = fun gps ->
    let uart = Unix.out_channel_of_descr !uart_mcu0 in
    let utm = utm_of WGS84 gps.wgs84 in
    Ubx.send uart nav_posutm
      ["EAST", scale utm.utm_x 1e2;
       "NORTH", scale utm.utm_y 1e2;
       "ALT", scale gps.alt 1e2];
    Ubx.send uart nav_status ["GPSfix", 3];
    Ubx.send uart nav_velned
      ["ITOW",scale gps.time 1e3;
       "VEL_D", -scale gps.climb 1e2;
       "GSpeed", scale gps.gspeed 1e2;
       "Heading", scale (deg_of_rad gps.course) 1e5]



  let infrared = fun ir_left ir_front ->
    let uart = Unix.out_channel_of_descr !uart_mcu0 in
    let ir_left = ir_left + ir_roll_neutral
    and ir_front = ir_front + ir_pitch_neutral in
    Ubx.send uart usr_irsim
      ["ROLL", ir_left;
       "PITCH", ir_front]

  let size_servos_buf = 256
  let zero = '\000'

  let get_2bytes = fun buf i ->
    (Char.code buf.[i] lsl 8) lor (Char.code buf.[i+1])

(* nb_servos 2 bytes values, prefixed by 00 ended by \n
   Returns optionaly a function associating the read value to the index *)
  let clock = 16
  let read_commands = fun servos ->
    let servos_buf = String.create size_servos_buf
    and buf_idx = ref 0 in
    let nb_servos = Array.length servos in
    let tty = Unix.in_channel_of_descr !uart_mcu1 in

    fun () ->
      let n = input tty servos_buf !buf_idx (size_servos_buf- !buf_idx) in
      let rec parse00 = fun i m ->
	if m >= 2+2*nb_servos+1 then 
	  (if servos_buf.[i] = zero then parse0 else parse00) (i+1) (m-1)
	else (* Not enough chars : wait *)
	  i

      and parse0 = fun i m ->
	if servos_buf.[i] = zero && servos_buf.[i+2*nb_servos+1] = '\n' then begin
	  for s = 0 to nb_servos - 1 do
	    servos.(s) <- get_2bytes servos_buf (i+1+2*s) / clock
	  done;
	  i+1+2*nb_servos+1
	end else (* 0 or \n missing *)
	  parse00 i m in

      let nb_available_chars = (!buf_idx + n) in
      let nb_read_chars = parse00 0 nb_available_chars in
      let rest = nb_available_chars - nb_read_chars in
      String.blit servos_buf nb_read_chars servos_buf 0 rest;
      buf_idx := rest



  let commands = fun commands ->
    ignore (GMain.Io.add_watch [`IN] (fun _ -> read_commands commands (); true) (GMain.Io.channel_of_descr !uart_mcu1))
end
