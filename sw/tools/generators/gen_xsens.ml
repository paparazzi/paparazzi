(*
 * XML preprocessing for UBX protocol
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

let out = stdout

exception Variable_data

let sizeof = function
"U32" -> 32
  | "U8" -> 8
  | "U4" | "I4" | "R4" -> 4
  | "U2" | "I2" -> 2
  | "U1" | "I1" -> 1
  | x -> failwith (sprintf "sizeof: unknown format '%s'" x)

let (+=) = fun r x -> r := !r + x

let test_type = function
"U32" | "U8" -> "skip"
  | "V" -> "variable"
  | _ -> "fixe"

let c_type = fun format ->
  match format with
      "R4" -> "float"
    | "I4" -> "int32_t"
    | "I2" -> "int16_t"
    | "I1" -> "int8_t"
    | "U4" -> "uint32_t"
    | "U2" -> "uint16_t"
    | "U1" -> "uint8_t"
    | _ -> failwith (sprintf "Gen_xsens.c_type: unknown format '%s'" format)

(* format is BigEndian *)
let get_at = fun offset format block_size ->
  let t = c_type format in
  let block_offset =
    if block_size = 0 then "" else sprintf "+%d*_xsens_block" block_size in
  match format with
      "R4" -> sprintf "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_xsens_payload+3+%d%s)|*((uint8_t*)_xsens_payload+2+%d%s)<<8|((uint32_t)*((uint8_t*)_xsens_payload+1+%d%s))<<16|((uint32_t)*((uint8_t*)_xsens_payload+%d%s))<<24); _f.f; })"  offset block_offset offset block_offset offset block_offset offset block_offset
    | "U4" | "I4" -> sprintf "(%s)(*((uint8_t*)_xsens_payload+3+%d%s)|*((uint8_t*)_xsens_payload+2+%d%s)<<8|((%s)*((uint8_t*)_xsens_payload+1+%d%s))<<16|((%s)*((uint8_t*)_xsens_payload+%d%s))<<24)" t offset block_offset offset block_offset t offset block_offset t offset block_offset
    | "U2" | "I2" -> sprintf "(%s)(*((uint8_t*)_xsens_payload+1+%d%s)|*((uint8_t*)_xsens_payload+%d%s)<<8)" t offset block_offset offset block_offset
    | "U1" | "I1" -> sprintf "(%s)(*((uint8_t*)_xsens_payload+%d%s))" t offset block_offset
    | _ -> failwith (sprintf "Gen_xsens.c_type: unknown format '%s'" format)

let define = fun x y ->
  fprintf out "#define %s %s\n" x y

exception Length_error of Xml.xml*int*int



let parse_message = fun m ->
  let msg_name = Xml.attrib m "name" in

  fprintf out "\n";
  let msg_id = sprintf "XSENS_%s_ID" msg_name in
  define msg_id (Xml.attrib m "ID");

  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  let offset = ref 0 in

  (** Generating read function *)
  let rec gen_read_macro = fun block_size f ->
    match Xml.tag f with
        "field" ->
          let fn = field_name f
          and fmt = format f  in
          begin
            match test_type (format f) with
                "fixe" ->
                  let block_no = if block_size = 0 then "" else ",_xsens_block" in
                  define (sprintf "XSENS_%s_%s(_xsens_payload%s)" msg_name fn block_no) (get_at !offset fmt block_size);
                  offset += sizeof fmt;
              | "variable" -> fprintf out "/* XSENS_%s_%s: variable data size */\n" msg_name fn;
              | _ -> offset += sizeof fmt;
          end
      | "block" ->
        let s = int_of_string (Xml.attrib f "length") in
        let o = !offset in
        List.iter (gen_read_macro s) (Xml.children f);
        let s' = !offset - o in
        if s <> s' then raise (Length_error (f, s, s'))
      | x -> failwith ("Unexpected field: " ^ x)
  in

  (** Generating send function *)
  let gen_send_macro = fun _ ->
    let param_name = fun f -> String.lowercase (field_name f) in
    let rec param_names = fun f r ->
      if Xml.tag f = "field" then
        param_name f :: r
      else
        List.fold_right param_names (Xml.children f) r in
    let param_type = fun f -> c_type (format f) in
    let get_msg_length = fun f ->
      try
        Xml.attrib f "length"
      with Xml.No_attribute("length") -> "0" in
    fprintf out "#define XSENS_%s(" msg_name;
    fprintf out "%s" (String.concat "," (param_names m []));
    fprintf out ") { \\\n";
    fprintf out "  XsensHeader(%s, %s);\\\n" msg_id (get_msg_length m);
    let rec send_one_field = fun f ->
      match Xml.tag f with
          "field" ->
            let s = sizeof (format f) in
            let p = param_name f in
            let t = param_type f in
            offset += sizeof (format f);
            fprintf out "  %s _%s = %s; XsensSend%dByAddr((uint8_t*)&_%s);\\\n" t p p s p
        | "block" ->
          List.iter send_one_field (Xml.children f)
        | _ -> assert (false) in
    List.iter send_one_field (Xml.children m);
    fprintf out "  XsensTrailer();\\\n";
    fprintf out "}"
  in

  let gen_access_macro =
    match Xml.attrib m "to" with
        "MT" -> gen_send_macro ();
      | "host" -> List.iter (gen_read_macro 0) (Xml.children m);
      | _ -> failwith "Unexpected direction";
  in
  gen_access_macro;

  begin
    try
      let l = int_of_string (Xml.attrib m "length") in
      if l <> !offset then raise (Length_error (m, l, !offset))
    with
        Xml.No_attribute("length") -> () (** Undefined length authorized *)
  end



let parse_data = fun d ->
  let data_name = Xml.attrib d "name" in

  fprintf out "\n";
  let data_length = sprintf "XSENS_DATA_%s_LENGTH" data_name in
  define data_length (Xml.attrib d "length");


  let field_name = fun f -> ExtXml.attrib f "name" in
  let format = fun f -> Xml.attrib f "format" in

  let offset = ref 0 in

  (** Generating read function *)
  let gen_read_macro = fun f ->
    match Xml.tag f with
        "field" ->
          let fn = field_name f
          and fdt = format f  in
          begin
            match test_type (format f) with
                "fixe" ->
            (* _xsens_block used as offset value *)
                  define (sprintf "XSENS_DATA_%s_%s(_xsens_payload,_xsens_block)" data_name fn) (get_at !offset fdt 1);
                  offset += sizeof fdt;
              | "variable" -> failwith (sprintf "XSENS_%s_%s: variable data size" data_name fn);
              | _ -> offset += sizeof fdt;
          end
      | x -> failwith ("Unexpected field: " ^ x)
  in

  List.iter gen_read_macro (Xml.children d);
  begin
    try
      let l = int_of_string (Xml.attrib d "length") in
      if l <> !offset then raise (Length_error (d, l, !offset))
    with
        Xml.No_attribute("length") -> () (** Undefined length authorized *)
  end

let parse_mask = fun m ->
  let mask_name = Xml.attrib m "name" in
  let mask = Xml.attrib m "bitmask" in
  let shift = ExtXml.attrib_or_default m "shift" "" in

  fprintf out "\n";
  define (sprintf "XSENS_MASK_%s(_conf)" mask_name) (sprintf "((_conf & %s)%s)" mask shift)


let parse_all = fun m ->
  match Xml.tag m with
      "message" -> parse_message m
    | "data" -> parse_data m
    | "mask" -> parse_mask m
    | x -> failwith (sprintf "Unexpected tag: %s" x)



let _ =
  if Array.length Sys.argv <> 2 then begin
    failwith (sprintf "Usage: %s <.xml xsens protocol file>" Sys.argv.(0))
  end;
  let xml_file = Sys.argv.(1) in
  try
    let xml = Xml.parse_file xml_file in
    fprintf out "/* Generated by gen_xsens from %s */\n" xml_file;
    fprintf out "/* Please DO NOT EDIT */\n\n";

    define "XSENS_START" "0xFA";
    define "XSENS_BID" "0xFF";
    fprintf out "\n";

    List.iter parse_all (Xml.children xml)
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
    | Dtd.Parse_error e ->
      fprintf stderr "File \"%s\", DTD parse error: %s\n" xml_file (Dtd.parse_error e)
    | Dtd.Check_error e ->
      fprintf stderr "File \"%s\", DTD check error: %s\n" xml_file (Dtd.check_error e)
    | Dtd.Prove_error e ->
      fprintf stderr "\nFile \"%s\", DTD prove error: %s\n\n" xml_file (Dtd.prove_error e)
