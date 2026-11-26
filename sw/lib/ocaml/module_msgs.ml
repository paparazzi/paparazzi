(*
 * Copyright (C) 2026 Freek van Tienen
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

(** Module specific messages: decode the MODULE_SPECIFIC telemetry message into a message
 * of the module_telemetry class of messages.xml, see sw/lib/python/module_msgs.py *)

(** Name of the carrier message (telemetry class) and of the class of the decoded messages *)
let carrier = "MODULE_SPECIFIC"
let class_name = "module_telemetry"

(** The decoder of the module_telemetry class, with its class id *)
type t = ((module PprzLink.MESSAGES) * int) option

(** The module_telemetry class of a messages.xml (protocol node), None if there is none *)
let of_xml = fun (xml:Xml.xml) : t ->
  try
    let c = ExtXml.child xml ~select:(fun x -> Xml.attrib x "name" = class_name) "msg_class" in
    let class_id = int_of_string (Xml.attrib c "id") in
    Some ((module PprzLink.MessagesOfXml (struct let xml = xml let name = class_name end) : PprzLink.MESSAGES), class_id)
  with _ -> None

(** The module_telemetry class of var/messages.xml *)
let default = lazy (of_xml (PprzLink.messages_xml ()))

(** true if [name] is a message of the module_telemetry class *)
let mem = fun (t:t) name ->
  match t with
  | None -> false
  | Some (m, _) ->
    let module M = (val m) in
    try ignore (M.message_of_name name); true with _ -> false

(** Decode the values of a MODULE_SPECIFIC message sent by [sender_id],
 * returns the decoded message and its values, None if the module is unknown or the data invalid *)
let decode = fun (t:t) sender_id (values:PprzLink.values) ->
  match t with
  | None -> None
  | Some (m, class_id) ->
    let module M = (val m) in
    try
      let module_id = PprzLink.int_assoc "module_id" values in
      let data = match PprzLink.assoc "data" values with
        | PprzLink.Array a -> Array.map PprzLink.int_of_value a
        | _ -> [||] in
      if not (Hashtbl.mem M.messages module_id) then raise Not_found;
      let msg = M.message_of_id module_id in
      (* a pprzlink payload of the module_telemetry class: sender, receiver, class, message id, then the data *)
      let b = Bytes.create (PprzLink.offset_fields + Array.length data) in
      Bytes.set b 0 (Char.chr (sender_id land 0xff));
      Bytes.set b 1 (Char.chr 0);
      Bytes.set b 2 (Char.chr (class_id land 0x0f));
      Bytes.set b 3 (Char.chr (module_id land 0xff));
      Array.iteri (fun i x -> Bytes.set b (PprzLink.offset_fields + i) (Char.chr (x land 0xff))) data;
      let (_, vs) = M.values_of_payload (Protocol.payload_of_bytes b) in
      Some (msg, vs)
    with _ -> None

(** Send a decoded message on Ivy, as a telemetry message from [sender] *)
let send = fun ?timestamp ?link_id (t:t) sender (msg:PprzLink.message) values ->
  match t with
  | None -> ()
  | Some (m, _) ->
    let module M = (val m) in
    M.message_send ?timestamp ?link_id sender msg.PprzLink.name values
