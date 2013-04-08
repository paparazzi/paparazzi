(*
 * Copyright (C) 2007- ENAC, Pascal Brisset, Antoine Drouin
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

(** Decode Data In Audio stream and print the messages on stdout *)

open Printf

module Sub_Pprz = Pprz.Messages(struct let name = "DIA" end)
module PprzTransport = Serial.Transport(Pprz.Transport)


let use_tele_message = fun buf ->
  let payload = Serial.payload_of_string buf in
  Debug.call 'l' (fun f ->  fprintf f "pprz receiving: %s\n" (Debug.xprint buf));
  try
    let (msg_id, ac_id, values) = Sub_Pprz.values_of_payload payload in
    let msg = Sub_Pprz.message_of_id msg_id in
    printf "%d %s\n%!" ac_id (Sub_Pprz.string_of_message msg values)
  with
      _ ->
        Debug.call 'W' (fun f ->  fprintf f "Warning, cannot use: %s\n" (Debug.xprint buf))


let _ =
  let port = ref "/dev/dsp" in
  let options = [
    "-d", Arg.Set_string port, (sprintf "<port> Default is %s" !port);
  ] in

  Arg.parse
    options
    (fun _x -> ())
    "Usage: ";

  (* Open the audio output and launch the periodic writings *)
  let fd = Hdlc.init_dec "/dev/dsp" in

  ignore (Glib.Io.add_watch [`IN] (fun _ -> use_tele_message (Hdlc.get_data ()); true) (GMain.Io.channel_of_descr fd));

  (* Main Loop *)
  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do
    ignore (Glib.Main.iteration true)
  done
