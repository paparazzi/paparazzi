(*
 * UBX protocol handling
 *
 * Copyright (C) 2004-2006 Pascal Brisset, Antoine Drouin
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

module UbxProtocol = struct
  (** SYNC1 SYNC2 CLASS ID LENGTH(2) UBX_PAYLOAD CK_A CK_B
      LENGTH is the lentgh of UBX_PAYLOAD
      For us, the 'payload' includes also CLASS, ID and the LENGTH *)
  let sync1 = Char.chr 0xb5
  let sync2 = Char.chr 0x62
  let offset_payload=2
  let offset_length=4
  let index_start = fun buf ->
    let rec loop = fun i ->
      let i' = String.index_from buf i sync1 in
      if String.length buf > i'+1 && buf.[i'+1] = sync2 then
        i'
      else
        loop (i'+1) in
    loop 0

  let payload_length = fun buf start ->
    Char.code buf.[start+5] lsl 8 + Char.code buf.[start+4] + 4

  let length = fun buf start ->
    let len = String.length buf - start in
    if len >= offset_length+2 then
      payload_length buf start + 4
    else
      raise Protocol.Not_enough

  let payload = fun buf ->
    Protocol.payload_of_string (String.sub buf offset_payload (payload_length buf 0))

  let uint8_t = fun x -> x land 0xff
  let (+=) = fun r x -> r := uint8_t (!r + x)
  let compute_checksum = fun buf ->
    let ck_a = ref 0 and ck_b = ref 0 in
    let l = String.length buf in
    for i = offset_payload to l - 1 - 4 do
      ck_a += Char.code buf.[i];
      ck_b += !ck_a
    done;
    (!ck_a, !ck_b)

  let checksum = fun buf->
    let (ck_a, ck_b) = compute_checksum buf in
    let l = payload_length buf 0 in
    ck_a = Char.code buf.[offset_payload+l+1] && ck_b = Char.code buf.[offset_payload+l+2]

  let packet = fun payload ->
    let payload = Protocol.string_of_payload payload in
    let n = String.length payload in
    let msg_length = n + 4 in
    let m = String.create msg_length in
    m.[0] <- sync1;
    m.[1] <- sync2;
    String.blit payload 0 m 2 n;
    let (ck_a, ck_b) = compute_checksum m in
    m.[msg_length-2] <- Char.chr ck_a;
    m.[msg_length-1] <- Char.chr ck_b;
    m
end

type class_id = int
type msg_id = int

let (//) = Filename.concat

let ubx_xml =
  lazy (ExtXml.parse_file (Env.paparazzi_src // "conf" // "ubx.xml"))

let ubx_get_class = fun name ->
  let ubx_xml = Lazy.force ubx_xml in
  ExtXml.child ubx_xml ~select:(fun x -> ExtXml.attrib x "name" = name) "msg_class"

let ubx_get_msg = fun ubx_class name ->
  ExtXml.child ubx_class ~select:(fun x -> ExtXml.attrib x "name" = name) "message"

let ubx_nav () = ubx_get_class "NAV"
let ubx_nav_id () = int_of_string (ExtXml.attrib (ubx_nav ()) "ID")

let ubx_usr () = ubx_get_class "USR"
let ubx_usr_id () = int_of_string (ExtXml.attrib (ubx_usr ()) "ID")

let ubx_get_nav_msg = fun name -> ubx_get_msg (ubx_nav ()) name
let ubx_get_usr_msg = fun name -> ubx_get_msg (ubx_usr ()) name

let nav_posutm () = ubx_nav_id (), ubx_get_nav_msg "POSUTM"
let nav_status () = ubx_nav_id (), ubx_get_nav_msg "STATUS"
let nav_velned () = ubx_nav_id (), ubx_get_nav_msg "VELNED"
let usr_irsim () = ubx_usr_id (), ubx_get_usr_msg "IRSIM"


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

type message_spec = Xml.xml

let ubx_payload = fun msg_xml values ->
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
        | "U1" ->
          assert(value >= 0 && value < 0x100);
          p.[pos] <- byte value
        | "I1" ->
          assert(value >= -0x80 && value <= 0x80);
          p.[pos] <- byte value
        | "I4" | "U4" ->
          assert(fmt <> "U4" || value >= 0);
          p.[pos+3] <- byte (value asr 24);
          p.[pos+2] <- byte (value lsr 16);
          p.[pos+1] <- byte (value lsr 8);
          p.[pos+0] <- byte value
        | "U2" | "I2" ->
          p.[pos+1] <- byte (value lsr 8);
          p.[pos+0] <- byte value
        | _ -> failwith (Printf.sprintf "Ubx.make_payload: unknown format '%s'" fmt)
    )
    values;
  p

let message = fun class_name msg_name ->
  let _class = ubx_get_class class_name in
  let class_id = int_of_string (ExtXml.attrib _class "ID") in
  let msg = ubx_get_msg _class msg_name in
  let msg_id = int_of_string (ExtXml.attrib msg "ID") in
  class_id, msg_id, msg


let payload = fun class_name msg_name values ->
  let class_id, msg_id, msg = message class_name msg_name in
  let u_payload = ubx_payload msg values in
  let n = String.length u_payload in

  (** Just add CLASS_ID, MSG_ID and LENGTH(2) to the ubx payload *)
  let m = String.create (n+4) in
  m.[0] <- Char.chr class_id;
  m.[1] <- Char.chr msg_id;
  m.[2] <- Char.chr (n land 0xff);
  m.[3] <- Char.chr ((n land 0xff00) lsr 8);
  String.blit u_payload 0 m 4 n;
  Protocol.payload_of_string m
