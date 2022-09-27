(*
 * Copyright (C) 2008 ENAC, Pascal Brisset, Antoine Drouin
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

(** Forward telemetry messages from the ivy bus to an UDP connection *)


let my_id = 0
module Tm_Pprz = PprzLink.Messages(struct let name = "telemetry" end)
module Dl_Pprz = PprzLink.Messages(struct let name = "datalink" end)
module PprzTransport = Protocol.Transport(Pprz_transport.Transport)


open Printf

let () =
  let ivy_bus = ref Defivybus.default_ivy_bus  in
  let host = ref "85.214.48.162"
  and port = ref 4242
  and datalink_port = ref 4243
  and id = ref "7" in

  let options = [
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-h", Arg.Set_string host, (sprintf "<remote host> Default is %s" !host);
    "-id", Arg.Set_string id , (sprintf "<aircraft id> Default is %s" !id);
    "-p", Arg.Set_int port, (sprintf "<remote port> Default is %d" !port);
    "-dp", Arg.Set_int datalink_port, (sprintf "<listening port> Default is %d" !datalink_port)
  ] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning: Discarding '%s'" x)
    "Usage: ";

  let addr = Unix.inet_addr_of_string !host in
  let sockaddr = Unix.ADDR_INET (addr, !port) in
  let socket = Unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in

  Ivy.init "Link" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  let get_ivy_message = fun _ args ->
    try
      let (msg_id, vs) = Tm_Pprz.values_of_string args.(0) in
      (* receiver_id unknown, using 0 instead... *)
      let payload = Tm_Pprz.payload_of_values msg_id (int_of_string !id) 0 vs in
      let buf = Pprz_transport.Transport.packet payload in
      let n = String.length buf in
      let n' = Unix.sendto socket (Bytes.of_string buf) 0 n [] sockaddr in
      assert (n = n')
    with _ -> () in

  let _b = Ivy.bind get_ivy_message (sprintf "^%s (.*)" !id) in

  (* Receiving a datalink message over UDP, on the same port *)
  let sockaddr = Unix.ADDR_INET (Unix.inet_addr_any, !datalink_port)
  and socket = Unix.socket Unix.PF_INET Unix.SOCK_DGRAM 0 in
  Unix.bind socket sockaddr;

  let buffer_size = 256 in
  let buffer = Bytes.create buffer_size in
  let get_datalink_message = fun _ ->
    begin
      try
        let n = input (Unix.in_channel_of_descr socket) buffer 0 buffer_size in
        let b = Bytes.sub buffer 0 n in
        Debug.trace 'x' (Debug.xprint (Bytes.to_string b));

        let use_dl_message = fun payload ->
          Debug.trace 'x' (Debug.xprint (Protocol.string_of_payload payload));
          let (header, values) = Dl_Pprz.values_of_payload payload in
          let msg = Dl_Pprz.message_of_id header.PprzLink.message_id in
          Dl_Pprz.message_send "ground_dl" msg.PprzLink.name values in

        assert (PprzTransport.parse use_dl_message (Bytes.to_string b) = n)
      with
          exc ->
            prerr_endline (Printexc.to_string exc)
    end;
    true in

  let ginput = GMain.Io.channel_of_descr socket in
  ignore (Glib.Io.add_watch [`IN] get_datalink_message ginput);

  (* Main Loop *)
  GMain.main ()
