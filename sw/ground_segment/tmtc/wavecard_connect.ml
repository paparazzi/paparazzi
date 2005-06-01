(*
 * $Id$
 *
 * Multi aircrafts receiver, logger and broadcaster
 *  
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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
module W = Wavecard
module Tc_Class = struct let name = "non" end
module Tc_Pprz = Pprz.Protocol(Tc_Class)


let send_ack = fun delay fd () ->
  ignore (GMain.Timeout.add delay (fun _ -> W.send fd (W.ACK, ""); false))

let send = fun fd a ->
  let (id, values) = Tc_Pprz.values_of_string a in
  let s  = Tc_Pprz.payload_of_values id values in
  Wavecard.send fd (W.REQ_SEND_FRAME,s)
    
let broadcast_msg = fun (com, data) ->
  match com with
    W.RECEIVED_FRAME ->
      let id, vs = Tc_Pprz.values_of_payload data in
      let m = Tc_Pprz.message_of_id id in
      let s = Tc_Pprz.string_of_message m vs in
      Ivy.send (sprintf "FROM_WAVECARD %s" s)
  | _ -> 
      Ivy.send (sprintf "RAW_FROM_WAVECARD %2x %s" (W.code_of_cmd com) data)

let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in
  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "Ivy bus (%s)" !ivy_bus);
      "-d", Arg.Set_string port, (sprintf "Port (%s)" !port)] in
  Arg.parse
    options
    (fun x -> ())
    "Usage: ";

  Ivy.init "Wavecard connect" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;
  
  try
    let fd = Serial.opendev !port Serial.B9600 in
    (* Listening *)
    let cb = fun _ ->
      Wavecard.receive ~ack:(send_ack 100 fd) broadcast_msg fd;
      true in

    ignore (Glib.Io.add_watch [`IN] cb (GMain.Io.channel_of_descr fd));
    
    (* Sending request from Ivy *)
    ignore (Ivy.bind (fun _ a -> send fd a.(0)) "TO_WAVECARD +(.*)");

    (* Main Loop *)
    let loop = Glib.Main.create true in
    while Glib.Main.is_running loop do
      ignore (Glib.Main.iteration true)
    done
  with
    exn -> fprintf stderr "%s\n" (Printexc.to_string exn)
      



