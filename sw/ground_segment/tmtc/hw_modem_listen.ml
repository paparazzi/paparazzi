(*
 * $Id$
 *
 * Hardware modem receiver
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

module ModemTransport = Serial.Transport(Modem.Protocol)
module Tele_Class = struct let name = "telemetry_ap" end
module Tele_Pprz = Pprz.Protocol(Tele_Class)
module PprzTransport = Serial.Transport(Tele_Pprz)

let use_pprz_message = fun ac_id (msg_id, values) ->
  let msg = Tele_Pprz.message_of_id msg_id in
  let s = String.concat " " (List.map (fun (_, v) -> Pprz.string_of_value v) values) in
  Ivy.send (sprintf "%d %s %s" ac_id msg.Pprz.name s)

let listen_pprz_modem = fun pprz_message_cb tty ->
  let fd = 
    if String.sub tty 0 4 = "/dev" then
      Serial.opendev tty Serial.B38400
    else
      Unix.descr_of_in_channel (open_in tty)
  in
  let use_pprz_buf = fun buf ->
    Debug.call 'P' (fun f -> fprintf f "use_pprz: %s\n" (Debug.xprint buf));
    pprz_message_cb (Tele_Pprz.values_of_bin buf) in
  let buffer = ref "" in
  let use_modem_message = fun msg ->
    Debug.call 'M' (fun f -> fprintf f "use_modem: %s\n" (Debug.xprint msg));
    match Modem.parse msg with
      None -> () (* Only internal modem data *)
    | Some data ->
	let b = !buffer ^ data in
	Debug.call 'M' (fun f -> fprintf f "Pprz buffer: %s\n" (Debug.xprint b));
	let x = PprzTransport.parse use_pprz_buf b in
	buffer := String.sub b x (String.length b - x)
  in
  let scanner = Serial.input (ModemTransport.parse use_modem_message) in
  let cb = fun _ ->
    begin
      try
	scanner fd
      with
	e -> fprintf stderr "%s\n" (Printexc.to_string e)
    end;
    true in
  
  ignore (Glib.Io.add_watch [`IN] cb (Glib.Io.channel_of_descr fd))

(* main loop *)
let _ =
  let ivy_bus = ref "127.255.255.255:2010" in
  let port = ref "/dev/ttyS0" in
  let ac_id = ref (-1) in
  let options =
    [ "-b", Arg.Set_string ivy_bus, (sprintf "Ivy bus (%s)" !ivy_bus);
      "-i", Arg.Set_int ac_id, "A/C id";
      "-d", Arg.Set_string port, (sprintf "Port (%s)" !port)] in
  Arg.parse
    options
    (fun x -> fprintf stderr "Warning:ignoring %s\n" x)
    "Usage: ";
  
  if !ac_id < 0 then
    failwith "A/C ic expected";

  Ivy.init "Paparazzi hw_modem_listen" "READY" (fun _ _ -> ());
  Ivy.start !ivy_bus;

  listen_pprz_modem (use_pprz_message !ac_id) !port;
  
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do
    ignore (Glib.Main.iteration true)
  done
