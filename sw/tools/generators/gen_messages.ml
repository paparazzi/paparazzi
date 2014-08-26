(*
 * XML preprocessing of messages.xml for downlink protocol
 *
 * Copyright (C) 2003-2008 ENAC, Pascal Brisset, Antoine Drouin
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

type format = string

type _type =
    Basic of string
  | Array of string * string
  | FixedArray of string * string * int

type field = _type  * string * format option

type fields = field list

type message = {
  name : string;
  id : int;
  period : float option;
  fields : fields
}

module Syntax = struct
  (** Parse a type name and returns a _type value *)
  let parse_type = fun t varname ->
    try
      let type_parts = Str.full_split (Str.regexp "[][]") t in
      match type_parts with
      | [Str.Text ty] -> Basic ty
      | [Str.Text ty; Str.Delim "["; Str.Delim "]"] -> Array (ty, varname)
      | [Str.Text ty; Str.Delim "["; Str.Text len ; Str.Delim "]"] -> FixedArray (ty, varname, int_of_string len)
      | _ -> failwith "Gen_messages: not a valid field type"
      with
      | Failure fail -> failwith("Gen_messages: not a valid array length")

  let length_name = fun s -> "nb_"^s

  let assoc_types t =
    try
      List.assoc t Pprz.types
    with
        Not_found ->
          failwith (sprintf "Error: '%s' unknown type" t)

  let rec sizeof = function
  Basic t -> string_of_int (assoc_types t).Pprz.size
    | Array (t, varname) -> sprintf "1+%s*%s" (length_name varname) (sizeof (Basic t))
    | FixedArray (t, varname, len) -> sprintf "0+%d*%s" len (sizeof (Basic t))

  let rec nameof = function
  Basic t -> String.capitalize t
    | Array _ -> failwith "nameof"
    | FixedArray _ -> failwith "nameof"

  (** Translates a "message" XML element into a value of the 'message' type *)
  let struct_of_xml = fun xml ->
    let name = ExtXml.attrib xml "name"
    and id = ExtXml.int_attrib xml "id"
    and period = try Some (ExtXml.float_attrib xml "period") with _ -> None
    and fields =
      List.map (fun field ->
        let id = ExtXml.attrib field "name"
        and type_name = ExtXml.attrib field "type"
        and fmt = try Some (Xml.attrib field "format") with _ -> None in
        let _type = parse_type type_name id in
        (_type, id, fmt))
      (List.filter (fun t -> compare (Xml.tag t) "field" = 0) (Xml.children xml)) in
    { id=id; name = name; period = period; fields = fields }

  let check_single_ids = fun msgs ->
    let tab = Array.create 256 false
    and  last_id = ref 0 in
    List.iter (fun msg ->
      if tab.(msg.id) then
        failwith (sprintf "Duplicated message id: %d" msg.id);
      if msg.id < !last_id then
        fprintf stderr "Warning: unsorted id: %d\n%!" msg.id;
      last_id := msg.id;
      tab.(msg.id) <- true)
      msgs

  (** Translates one class of a XML message file into a list of messages *)
  let read = fun filename class_ ->
    let xml = Xml.parse_file filename in
    try
      let xml_class = ExtXml.child ~select:(fun x -> Xml.attrib x "name" = class_) xml "msg_class" in
      let msgs = List.map struct_of_xml (Xml.children xml_class) in
      check_single_ids msgs;
      msgs
    with
        Not_found -> failwith (sprintf "No msg_class '%s' found" class_)
end (* module Suntax *)


(** Pretty printer of C macros for sending and parsing messages *)
module Gen_onboard = struct
  let print_field = fun h (t, name, (_f: format option)) ->
    match t with
        Basic _ ->
          fprintf h "\t  DownlinkPut%sByAddr(_trans, _dev, (%s)); \\\n" (Syntax.nameof t) name
      | Array (t, varname) ->
        let _s = Syntax.sizeof (Basic t) in
        fprintf h "\t  DownlinkPut%sArray(_trans, _dev, %s, %s); \\\n" (Syntax.nameof (Basic t)) (Syntax.length_name varname) name
      | FixedArray (t, varname, len) ->
        let _s = Syntax.sizeof (Basic t) in
        fprintf h "\t  DownlinkPut%sFixedArray(_trans, _dev, %d, %s); \\\n" (Syntax.nameof (Basic t)) len name

  let print_parameter h = function
      (Array _, s, _) -> fprintf h "%s, %s" (Syntax.length_name s) s
    | (FixedArray _, s, _) -> fprintf h "%s" s
    | (_, s, _) -> fprintf h "%s" s

  let print_macro_parameters h = function
  [] -> ()
    | f::fields ->
      print_parameter h f;
      List.iter (fun f -> fprintf h ", "; print_parameter h f) fields

  let rec size_fields = fun fields size ->
    match fields with
        [] -> size
      | (t, _, _)::fields -> size_fields fields (size ^"+"^Syntax.sizeof t)

  let size_of_message = fun m -> size_fields m.fields "0"

  let estimated_size_of_message = fun m ->
    try
      List.fold_right
        (fun (t, _, _)  r ->  int_of_string (Syntax.sizeof t)+r)
        m.fields
        0
    with
        Failure "int_of_string" -> 0

  let print_downlink_macro = fun h {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "#define DOWNLINK_SEND_%s(_trans, _dev, " s;
    end else
      fprintf h "#define DOWNLINK_SEND_%s(_trans, _dev " s;
    print_macro_parameters h fields;
    fprintf h "){ \\\n";
    let size = (size_fields fields "0") in
    fprintf h "\tif (DownlinkCheckFreeSpace(_trans, _dev, DownlinkSizeOf(_trans, _dev, %s))) {\\\n" size;
    fprintf h "\t  DownlinkCountBytes(_trans, _dev, DownlinkSizeOf(_trans, _dev, %s)); \\\n" size;
    fprintf h "\t  DownlinkStartMessage(_trans, _dev, \"%s\", DL_%s, %s) \\\n" s s size;
    List.iter (print_field h) fields;
    fprintf h "\t  DownlinkEndMessage(_trans, _dev ) \\\n";
    fprintf h "\t} else \\\n";
    fprintf h "\t  DownlinkOverrun(_trans, _dev ); \\\n";
    fprintf h "}\n\n"

  let print_null_downlink_macro = fun h {name=s; fields = fields} ->
    if List.length fields > 0 then begin
      fprintf h "#define DOWNLINK_SEND_%s(_trans, _dev, " s;
    end else
      fprintf h "#define DOWNLINK_SEND_%s(_trans, _dev " s;
    print_macro_parameters h fields;
    fprintf h ") {}\n"

  (** Prints the messages ids *)
  let print_enum = fun h class_ messages ->
    List.iter (fun m ->
      if m.id < 0 || m.id > 255 then begin
        fprintf stderr "Error: message %s has id %d but should be between 0 and 255\n" m.name m.id; exit 1;
      end
      else fprintf h "#define DL_%s %d\n" m.name m.id
    ) messages;
    fprintf h "#define DL_MSG_%s_NB %d\n\n" class_ (List.length messages)

  (** Prints the table of the messages lengths *)
  let print_lengths_array = fun h class_ messages ->
    let sizes = List.map (fun m -> (m.id, size_of_message m)) messages in
    let max_id = List.fold_right (fun (id, _m) x -> max x id) sizes min_int in
    let n = max_id + 1 in
    fprintf h "#define MSG_%s_LENGTHS {" class_;
    for i = 0 to n - 1 do
      fprintf h "%s," (try "(2+" ^ List.assoc i sizes^")" with Not_found -> "0")
    done;
    fprintf h "}\n\n";

    (* Print a comment with the actual size (when not variable) *)
    fprintf h "/*\n Size for non variable messages\n";

    let sizes =
      List.map
        (fun m -> (estimated_size_of_message m, m.name))
        messages in
    let sizes = List.sort (fun (s1,_) (s2,_) -> compare s2 s1) sizes in

    List.iter
      (fun (s, id) -> fprintf h "%2d : %s\n" s id)
      sizes;
    fprintf h "*/\n"

  (** Prints the macros required to send a message *)
  let print_downlink_macros = fun h class_ messages ->
    print_enum h class_ messages;
    print_lengths_array h class_ messages;
    List.iter (print_downlink_macro h) messages

  let print_null_downlink_macros = fun h messages ->
    List.iter (print_null_downlink_macro h) messages

  (** Prints the macro to get access to the fields of a received message *)
  let print_get_macros = fun h check_alignment message ->
    let msg_name = message.name in
    let offset = ref Pprz.offset_fields in

    (** Prints the macro for one field, using the global [offset] ref *)
    let parse_field = fun (_type, field_name, _format) ->
      if !offset < 0 then
        failwith "FIXME: No field allowed after an array field (print_get_macros)";
      (** Converts bytes into the required type *)
      let typed = fun o pprz_type -> (* o for offset *)
        let size = pprz_type.Pprz.size in
        if check_alignment && o mod (min size 4) <> 0 then
          failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);

        match size with
            1 -> sprintf "(%s)(*((uint8_t*)_payload+%d))" pprz_type.Pprz.inttype o
          | 2 -> sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8)" pprz_type.Pprz.inttype o o
          | 4 when pprz_type.Pprz.inttype = "float" ->
            sprintf "({ union { uint32_t u; float f; } _f; _f.u = (uint32_t)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24); _f.f; })" o o o o
          | 8 when pprz_type.Pprz.inttype = "double" ->
            let s = ref (sprintf "*((uint8_t*)_payload+%d)" o) in
            for i = 1 to 7 do
              s := !s ^ sprintf "|((uint64_t)*((uint8_t*)_payload+%d+%d))<<%d" o i (8*i)
            done;

            sprintf "({ union { uint64_t u; double f; } _f; _f.u = (uint64_t)(%s); Swap32IfBigEndian(_f.u); _f.f; })" !s
          | 4 ->
            sprintf "(%s)(*((uint8_t*)_payload+%d)|*((uint8_t*)_payload+%d+1)<<8|((uint32_t)*((uint8_t*)_payload+%d+2))<<16|((uint32_t)*((uint8_t*)_payload+%d+3))<<24)" pprz_type.Pprz.inttype o o o o
          | 8 ->
            let s = ref (sprintf "(%s)(*((uint8_t*)_payload+%d)" pprz_type.Pprz.inttype o) in
            for i = 1 to 7 do
              s := !s ^ sprintf "|((uint64_t)*((uint8_t*)_payload+%d+%d))<<%d" o i (8*i)
            done;
            sprintf "%s)" !s
          | _ -> failwith "unexpected size in Gen_messages.print_get_macros" in

      (** To be an array or not to be an array: *)
      match _type with
          Basic t ->
            let pprz_type = Syntax.assoc_types t in
            fprintf h "#define DL_%s_%s(_payload) (%s)\n" msg_name field_name (typed !offset pprz_type);
            offset := !offset + pprz_type.Pprz.size

        | Array (t, _varname) ->
      (** The macro to access to the length of the array *)
          fprintf h "#define DL_%s_%s_length(_payload) (%s)\n" msg_name field_name (typed !offset (Syntax.assoc_types "uint8"));
          incr offset;

      (** The macro to access to the array itself *)
          let pprz_type = Syntax.assoc_types t in
          if check_alignment && !offset mod (min pprz_type.Pprz.size 4) <> 0 then
            failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);

          fprintf h "#define DL_%s_%s(_payload) ((%s*)(_payload+%d))\n" msg_name field_name pprz_type.Pprz.inttype !offset;
          offset := -1 (** Mark for no more fields *)
        | FixedArray (t, _varname, len) ->
            (** The macro to access to the length of the array *)
            fprintf h "#define DL_%s_%s_length(_payload) (%d)\n" msg_name field_name len;
            (** The macro to access to the array itself *)
            let pprz_type = Syntax.assoc_types t in
            if check_alignment && !offset mod (min pprz_type.Pprz.size 4) <> 0 then
              failwith (sprintf "Wrong alignment of field '%s' in message '%s" field_name msg_name);

            fprintf h "#define DL_%s_%s(_payload) ((%s*)(_payload+%d))\n" msg_name field_name pprz_type.Pprz.inttype !offset;
            offset := !offset + (pprz_type.Pprz.size*len)
    in

    fprintf h "\n";
    (** Do it for all the fields of the message *)
    List.iter parse_field message.fields

end (* module Gen_onboard *)


(********************* Main **************************************************)
let () =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0))
  end;

  let filename = Sys.argv.(1)
  and class_name = Sys.argv.(2) in

  try
    let messages = Syntax.read filename class_name in

    let h = stdout in

    Printf.fprintf h "/* Automatically generated from %s */\n" filename;
    Printf.fprintf h "/* Please DO NOT EDIT */\n";

    Printf.fprintf h "/* Macros to send and receive messages of class %s */\n" class_name;

    (** Macros for airborne downlink (sending) *)
    if class_name = "telemetry" then begin (** FIXME *)
      Printf.fprintf h "#ifdef DOWNLINK\n"
    end;
    Gen_onboard.print_downlink_macros h class_name messages;
    if class_name = "telemetry" then begin
      Printf.fprintf h "#else // DOWNLINK\n";
      Gen_onboard.print_null_downlink_macros h messages;
      Printf.fprintf h "#endif // DOWNLINK\n"
    end;

    (** Macros for airborne datalink (receiving) *)
    let check_alignment = class_name <> "telemetry" in
    List.iter (Gen_onboard.print_get_macros h check_alignment) messages

  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
