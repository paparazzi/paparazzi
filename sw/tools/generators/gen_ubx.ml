(*
 * XML preprocessing for UBX protocol
 *
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
 * Copyright (C) 2015 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

let out = stdout

let sizeof = function
  | "R8" -> 8
  | "U4" | "I4" | "R4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let c_type = fun format ->
  match format with
    | "R8" -> "double"
    | "R4" -> "float"
    | "I4" -> "int32_t"
    | "U4" -> "uint32_t"
    | "I2" -> "int16_t"
    | "U2" -> "uint16_t"
    | "I1" -> "int8_t"
    | "U1" -> "uint8_t"
    | _ -> failwith (sprintf "Gen_ubx.c_type: unknown format '%s'" format)

let get_at = fun offset format block_size ->
  let t = c_type format in
  let block_offset =
    if block_size = 0 then "" else sprintf "+%d*_ubx_block" block_size in
  match format with
    | "R8" ->
        let s = ref (sprintf "*((uint8_t*)_ubx_payload+%d%s)" offset block_offset) in
        for i = 1 to 7 do
          s := !s ^ sprintf "|((uint64_t)*((uint8_t*)_ubx_payload+%d+%d%s))<<%d" i offset block_offset (8*i)
        done;
        sprintf "({ union { uint64_t u; double f; } _f; _f.u = (uint64_t)(%s); /*Swap32IfBigEndian(_f.u)*/; _f.f; })" !s
    | "R4" -> sprintf "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_ubx_payload+%d%s)|*((uint8_t*)_ubx_payload+1+%d%s)<<8|((uint32_t)*((uint8_t*)_ubx_payload+2+%d%s))<<16|((uint32_t)*((uint8_t*)_ubx_payload+3+%d%s))<<24); _f.f; })" offset block_offset offset block_offset offset block_offset offset block_offset
    | "U4" | "I4" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s)|*((uint8_t*)_ubx_payload+1+%d%s)<<8|((%s)*((uint8_t*)_ubx_payload+2+%d%s))<<16|((%s)*((uint8_t*)_ubx_payload+3+%d%s))<<24)" t offset block_offset offset block_offset t offset block_offset t offset block_offset
    | "U2" | "I2" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s)|*((uint8_t*)_ubx_payload+1+%d%s)<<8)" t offset block_offset offset block_offset
    | "U1" | "I1" -> sprintf "(%s)(*((uint8_t*)_ubx_payload+%d%s))" t offset block_offset
    | _ -> failwith (sprintf "Gen_ubx.c_type: unknown format '%s'" format)

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int




let parse_message = fun class_name m ->
  let msg_name = Xml.attrib m "name" in

  fprintf out "\n";
  let msg_id = sprintf "UBX_%s_%s_ID" class_name msg_name in
  define msg_id (Xml.attrib m "ID");

  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  let offset = ref 0 in
  let rec gen_access_macro = fun block_size f ->
    match Xml.tag f with
        "field" ->
          let fn = field_name f
          and fmt = format f  in
          let block_no = if block_size = 0 then "" else ",_ubx_block" in
          define (sprintf "UBX_%s_%s_%s(_ubx_payload%s)" class_name msg_name fn block_no) (get_at !offset fmt block_size);
          offset += sizeof fmt
      | "block" ->
        let s = int_of_string (Xml.attrib f "length") in
        let o = !offset in
        List.iter (gen_access_macro s) (Xml.children f);
        let s' = !offset - o in
        if s <> s' then raise (Length_error (f, s, s'))
      | x -> failwith ("Unexpected field: " ^ x)
  in

  List.iter (gen_access_macro 0) (Xml.children m);
  begin
    try
      let l = int_of_string (Xml.attrib m "length") in
      if l <> !offset then raise (Length_error (m, l, !offset))
    with
        Xml.No_attribute("length") -> () (** Undefined length authorized *)
  end;

  (** Generating send function *)
  let param_type = fun f -> c_type (format f) in
  let param_name = fun f ->String.lowercase (field_name f) in
  let param_name_and_type = fun f ->
    sprintf "%s ubx_%s" (param_type f) (param_name f) in
  let rec param_names = fun f r ->
    if Xml.tag f = "field" then
      param_name_and_type f :: r
    else
      List.fold_right param_names (Xml.children f) r in
  fprintf out "\nstatic inline void UbxSend_%s_%s(" class_name msg_name;
  fprintf out "%s" (String.concat ", " (["struct link_device *dev"] @ (param_names m [])));
  fprintf out ") {\n";
  fprintf out "  ubx_header(dev, UBX_%s_ID, %s, %d);\n" class_name msg_id !offset;
  let rec send_one_field = fun f ->
    match Xml.tag f with
        "field" ->
          let s = sizeof (format f) in
          let p = param_name f in
          let t = param_type f in
          fprintf out "  %s _%s = ubx_%s; ubx_send_bytes(dev, %d, (uint8_t*)&_%s);\n" t p p s p
      | "block" ->
        List.iter send_one_field (Xml.children f)
      | _ -> assert (false) in
  List.iter send_one_field (Xml.children m);
  fprintf out "  ubx_trailer(dev);\n";
  fprintf out "}\n"


let parse_class = fun c ->
  let _class_id = int_of_string (Xml.attrib c "id")
  and class_name = Xml.attrib c "name" in

  fprintf out "\n";
  define (sprintf "UBX_%s_ID" class_name) (Xml.attrib c "ID");

  List.iter (parse_message class_name) (Xml.children c)


let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml ubx protocol file>" Sys.argv.(0))
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated by gen_ubx from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";

    fprintf out "#include \"pprzlink/pprzlink_device.h\"\n\n";
    fprintf out "#include \"subsystems/gps/gps_ubx.h\"\n\n";

    define "UBX_SYNC1" "0xB5";
    define "UBX_SYNC2" "0x62";

    List.iter parse_class (Xml.children xml)
  with
      Xml.Error (em, ep) ->
        let l = Xml.line ep
        and c1, c2 = Xml.range ep in
        fprintf stderr "File \"%s\", line %d, characters %d-%d:\n" xml_file l c1 c2;
        fprintf stderr "%s\n" (Xml.error_msg em);
        exit 1
    | Length_error (m, l1, l2) ->
      fprintf stderr "File \"%s\", inconsistent length: %d expected, %d found from fields in message:\n %s\n" xml_file l1 l2 (Xml.to_string_fmt m);
      exit 1
    | Dtd.Check_error e ->
      fprintf stderr "File \"%s\", DTD check error: %s\n" xml_file (Dtd.check_error e)
    | Dtd.Prove_error e ->
      fprintf stderr "\nFile \"%s\", DTD check error: %s\n\n" xml_file (Dtd.prove_error e)
