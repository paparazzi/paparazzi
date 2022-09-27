(*
 * Copyright (C) 2008 ENAC, Pascal Brisset, Antoine Drouin
 * Copyright (C) 2016 Gautier Hattenberger <gautier.hattenberger@enac.fr>
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

(**
 *  Forward telemetry messages from the ivy bus to a serial connection
 *  a.k.a. a "reverse" link agent:
 *    ivy telemetry message -> serial out
 *    serial in -> ivy datalink message
 *
 *  This tool can be used to rebuild the airborne modem input/output
 *  from the Ocaml simulator that directly outputs Ivy messages
 *
 *  Only the PPRZ message format is supported
 *)


module Tm_Pprz = PprzLink.Messages(struct let name = "telemetry" end)
module Dl_Pprz = PprzLink.Messages(struct let name = "datalink" end)
module PprzTransport = Protocol.Transport(Pprz_transport.Transport)


open Printf

let () =

  (* Options *)
  let ivy_bus = ref Defivybus.default_ivy_bus  in
  let device = ref "/dev/ttyUSB0"
  and baudrate = ref "57600"
  and id = ref "1" in

  let options = [
    "-id", Arg.Set_string id , (sprintf "<AC id> Default is %s" !id);
    "-b", Arg.Set_string ivy_bus, (sprintf "<ivy bus> Default is %s" !ivy_bus);
    "-d", Arg.Set_string device, (sprintf "<serial device> Default is %s" !device);
    "-baudrate", Arg.Set_string baudrate, (sprintf "<serial baudrate> Default is %s" !baudrate);
  ] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning: Discarding '%s'" x)
    "Usage: ";

  (* Open serial device *)
  let fd = Serial.opendev !device (Serial.speed_of_baudrate !baudrate) false in

  (* Start Ivy *)
  Ivy.init "ivy2serial" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  (* Bind to all Ivy message from AC (telemetry) and send over serial link *)
  let get_ivy_message = fun _ args ->
    try
      let (msg_id, vs) = Tm_Pprz.values_of_string args.(0) in
      (* destination is always 0 (ground) ? *)
      let payload = Tm_Pprz.payload_of_values msg_id (int_of_string !id) 0 vs in
      let buf = Pprz_transport.Transport.packet payload in
      let o = Unix.out_channel_of_descr fd in
      Printf.fprintf o "%s" buf; flush o
    with _ -> () in

  let _b = Ivy.bind get_ivy_message (sprintf "^%s (.*)" !id) in

  (* The function to be called when data is available *)
  let buffer_size = 256 in
  let buffer = Bytes.create buffer_size in
  let get_datalink_message = fun _ ->
    begin
      try
        let n = input (Unix.in_channel_of_descr fd) buffer 0 buffer_size in
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

  (* Connect IO watch *)
  let hangup = fun _ ->
    prerr_endline "Serial link disconnected";
    exit 1 in
  ignore (Glib.Io.add_watch [`HUP] hangup (GMain.Io.channel_of_descr fd));
  ignore (Glib.Io.add_watch [`IN] get_datalink_message (GMain.Io.channel_of_descr fd));

  (* Main Loop *)
  GMain.main ()
