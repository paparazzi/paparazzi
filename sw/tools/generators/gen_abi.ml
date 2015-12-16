(*
 * XML preprocessing of messages.xml for aiborne middleware ABI
 *
 * Copyright (C) 2011 ENAC, Gautier Hattenberger
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

type _type = string
type _name = string

type field = _name * _type

type fields = field list

type message = {
  name : string;
  id : int;
  fields : fields
}

module Syntax = struct
  (** Translates a "message" XML element into a value of the 'message' type *)
  let struct_of_xml = fun xml ->
    let name = ExtXml.attrib xml "name"
    and id = ExtXml.int_attrib xml "id"
    and fields =
      List.map
        (fun field ->
          let _name = ExtXml.attrib field "name"
          and _type = ExtXml.attrib field "type" in
          (_name, _type))
        (Xml.children xml) in
    { id = id; name = name; fields = fields }

  let check_single_ids = fun msgs ->
    let tab = Array.make 256 false (* TODO remove limitation to 256 msg not needed here *)
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


(** Pretty printer *)
module Gen_onboard = struct
  (* Print message IDs and return the highest value *)
  let print_message_id = fun h messages ->
    let highest_id = ref 0 in
    Printf.fprintf h "\n/* Messages IDs */\n";
    List.iter (fun msg ->
      if msg.id > !highest_id then highest_id := msg.id;
      Printf.fprintf h "#define ABI_%s_ID %d\n" (String.capitalize msg.name) msg.id
    ) messages;
    !highest_id

  (* Print structure array *)
  let print_struct = fun h size ->
    Printf.fprintf h "\n/* Array and linked list structure */\n";
    Printf.fprintf h "#define ABI_MESSAGE_NB %d\n\n" (size+1);
    Printf.fprintf h "ABI_EXTERN abi_event* abi_queues[ABI_MESSAGE_NB];\n"

  (* Print arguments' function from fields *)
  let print_args = fun h fields ->
    let rec args = fun h l ->
      match l with
          [] -> Printf.fprintf h ")"
        | [(n,t)] -> Printf.fprintf h ", %s %s)" t n
        | (n,t)::l' -> Printf.fprintf h ", %s %s" t n; args h l'
    in
    Printf.fprintf h "(uint8_t sender_id";
    args h fields

  (* Print callbacks prototypes for all messages *)
  let print_callbacks = fun h messages ->
    Printf.fprintf h "\n/* Callbacks */\n";
    List.iter (fun msg ->
      Printf.fprintf h "typedef void (*abi_callback%s)" (String.capitalize msg.name);
      print_args h msg.fields;
      Printf.fprintf h ";\n";
    ) messages

  (* Print a bind function *)
  let print_msg_bind = fun h msg ->
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiBindMsg%s(uint8_t sender_id, abi_event * ev, abi_callback%s cb) {\n" name name;
    Printf.fprintf h "  ev->id = sender_id;\n";
    Printf.fprintf h "  ev->cb = (abi_callback)cb;\n";
    Printf.fprintf h "  ABI_PREPEND(abi_queues[ABI_%s_ID],ev);\n" name;
    Printf.fprintf h "}\n"

  (* Print a send function *)
  let print_msg_send = fun h msg ->
    (* print arguments *)
    let rec args = fun h l ->
      match l with
          [] -> Printf.fprintf h ");\n"
        | [(n,_)] -> Printf.fprintf h ", %s);\n" n
        | (n,_)::l' -> Printf.fprintf h ", %s" n; args h l'
    in
    let name = String.capitalize msg.name in
    Printf.fprintf h "\nstatic inline void AbiSendMsg%s" name;
    print_args h msg.fields;
    Printf.fprintf h " {\n";
    Printf.fprintf h "  abi_event* e;\n";
    Printf.fprintf h "  ABI_FOREACH(abi_queues[ABI_%s_ID],e) {\n" name;
    Printf.fprintf h "    if (e->id == ABI_BROADCAST || e->id == sender_id) {\n";
    Printf.fprintf h "      abi_callback%s cb = (abi_callback%s)(e->cb);\n" name name;
    Printf.fprintf h "      cb(sender_id";
    args h msg.fields;
    Printf.fprintf h "    }\n";
    Printf.fprintf h "  }\n";
    Printf.fprintf h "}\n"

  (* Print bind and send functions for all messages *)
  let print_bind_send = fun h messages ->
    Printf.fprintf h "\n/* Bind and Send functions */\n";
    List.iter (fun msg ->
      print_msg_bind h msg;
      print_msg_send h msg
    ) messages

end (* module Gen_onboard *)


(********************* Main **************************************************)
let () =
  if Array.length Sys.argv <> 3 then begin
    failwith (sprintf "Usage: %s <.xml file> <class_name>" Sys.argv.(0))
  end;

  let filename = Sys.argv.(1)
  and class_name = Sys.argv.(2) in

  try
    let h = stdout in

    (** Read and store messages *)
    let messages = Syntax.read filename class_name in

    (** Print file header *)
    Printf.fprintf h "/* Automatically generated by gen_abi from %s */\n" filename;
    Printf.fprintf h "/* Version %s */\n" (Env.get_paparazzi_version ());
    Printf.fprintf h "/* Please DO NOT EDIT */\n\n";
    Printf.fprintf h "/* Onboard middleware library ABI\n";
    Printf.fprintf h " * send and receive messages of class %s\n" class_name;
    Printf.fprintf h " */\n\n";
    Printf.fprintf h "#ifndef ABI_MESSAGES_H\n";
    Printf.fprintf h "#define ABI_MESSAGES_H\n\n";
    Printf.fprintf h "#include \"subsystems/abi_common.h\"\n";

    (** Print Messages IDs *)
    let highest_id = Gen_onboard.print_message_id h messages in

    (** Print general structure definition *)
    Gen_onboard.print_struct h highest_id;

    (** Print Messages callbacks definition *)
    Gen_onboard.print_callbacks h messages;

    (** Print Bind and Send functions for all messages *)
    Gen_onboard.print_bind_send h messages;

    Printf.fprintf h "\n#endif // ABI_MESSAGES_H\n"
  with
      Xml.Error (msg, pos) -> failwith (sprintf "%s:%d : %s\n" filename (Xml.line pos) (Xml.error_msg msg))
