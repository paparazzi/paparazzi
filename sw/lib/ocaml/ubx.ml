(*
 * $Id$
 *
 * UBX protocol handling
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec, Pascal Brisset, Antoine Drouin
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
module Protocol = struct
  let index_start = fun buf ->
    let rec loop = fun i ->
      let i' = String.index_from buf i (Char.chr 0xb5) in
      if String.length buf > i'+1 && buf.[i'+1] = Char.chr 0x62 then
	i'
      else
	loop (i'+1) in
    loop 0

  let payload_length = fun buf start ->
    Char.code buf.[start+5] lsl 8 + Char.code buf.[start+4]

  let length = fun buf start ->
    let len = String.length buf - start in
    if len > 6 then
      payload_length buf start + 8
    else
      raise Serial.Not_enough

  let payload = fun buf start ->
    String.sub buf (start+6) (payload_length buf start)

  let uint8_t = fun x -> x land 0xff
  let (+=) = fun r x -> r := uint8_t (!r + x)
  let checksum = fun buf start payload ->
    let ck_a = ref 0 and ck_b = ref 0 in
    let l = String.length payload in
    for i = 0 to l - 1 do
      ck_a += Char.code payload.[i];
      ck_b += !ck_a
    done;
    !ck_a = Char.code buf.[start+l+6] && !ck_b = Char.code buf.[start+l+7]
end

let (//) = Filename.concat

let ubx_xml =
  Xml.parse_file (Env.paparazzi_src // "conf" // "ubx.xml")

let ubx_get_class = fun name ->
  ExtXml.child ubx_xml ~select:(fun x -> ExtXml.attrib x "name" = name) "class"

let ubx_nav = ubx_get_class "NAV"
let ubx_nav_id = int_of_string (ExtXml.attrib ubx_nav "ID")
let ubx_get_msg = fun ubx_class name ->
  ExtXml.child ubx_class ~select:(fun x -> ExtXml.attrib x "name" = name) "message"

let ubx_get_nav_msg = fun name -> ubx_get_msg ubx_nav name

let nav_posllh = ubx_nav_id, ubx_get_nav_msg "POSLLH"
let nav_posutm = ubx_nav_id, ubx_get_nav_msg "POSUTM"
let nav_status = ubx_nav_id, ubx_get_nav_msg "STATUS"
let nav_velned = ubx_nav_id, ubx_get_nav_msg "VELNED"


let send_start_sequence = fun gps ->
  output_byte gps 0xB5;
  output_byte gps 0x62


let sizeof = function
    "U4" | "I4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (Printf.sprintf "Ubx.sizeof: unknown format '%s'" x)

let assoc = fun label fields ->
  let rec loop o = function
      [] -> raise Not_found
    | f::fs ->
	let format = ExtXml.attrib f "format" in
	if ExtXml.attrib f "name" = label
	then (o, format)
	else loop (o + sizeof format) fs in
  loop 0 fields

let byte = fun x -> Char.chr (x land 0xff)

let make_payload = fun msg_xml values ->
  let n = int_of_string (ExtXml.attrib msg_xml "length") in
  let p = String.make n '#' in
  let fields = Xml.children msg_xml in
  List.iter
    (fun (label, value) ->
      let (pos, fmt) =
	try
	  assoc label fields
	with
	  Not_found -> failwith (Printf.sprintf "Field '%s' not found in %s" label (Xml.to_string msg_xml))
      in
      match fmt with
      |	"U1" ->
	  assert(value >= 0 && value < 0x100);
	  p.[pos] <- byte value
      |	"I4" | "U4" ->
	  assert(fmt <> "U4" || value >= 0);
	  p.[pos+3] <- byte (value asr 24);
	  p.[pos+2] <- byte (value lsr 16);
	  p.[pos+1] <- byte (value lsr 8);
	  p.[pos+0] <- byte value
      |	"U2" | "I2" ->
	  p.[pos+1] <- byte (value lsr 8);
	  p.[pos+0] <- byte value
      |	_ -> failwith (Printf.sprintf "Ubx.make_payload: unknown format '%s'" fmt)
    )
    values;
  p
  
  



let send = fun gps (msg_class, msg) values ->
  let msg_id = int_of_string (Xml.attrib msg "ID") in
  let payload = make_payload msg values in
  let n = String.length payload in
  send_start_sequence gps;

  let ck_a = ref 0 and ck_b = ref 0 in
  let output_byte_ck = fun c -> 
    ck_a := (!ck_a+c) land 0xff; ck_b := (!ck_b+ !ck_a) land 0xff;
    output_byte gps c in

  output_byte_ck msg_class;
  output_byte_ck msg_id;
  output_byte_ck (n land 0xff);
  output_byte_ck ((n land 0xff00) lsr 8);
  String.iter (fun c -> output_byte_ck (Char.code c)) payload;
  output_byte gps !ck_a;
  output_byte gps !ck_b;
  flush gps


