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


let _ =
  if Array.length Sys.argv <> 3 then
    failwith (sprintf "Usage: %s <message class> <port>" Sys.argv.(0));

  let class_name = Sys.argv.(1) in

  let module Tele_Class = struct let name = class_name end in
  let module Tele_Pprz = Pprz.Protocol(Tele_Class) in
  let module PprzTransport = Serial.Transport(Tele_Pprz) in

  let listen_tty = fun use_pprz_message tty ->
    let fd = Serial.opendev tty Serial.B4800 in

    let use_pprz_buf = fun buf ->
      use_pprz_message (Tele_Pprz.values_of_bin buf) in

    let scanner = Serial.input (PprzTransport.parse use_pprz_buf) in
    let cb = fun _ ->
      begin
	try
	  scanner fd
	with
	  e -> fprintf stderr "%s\n" (Printexc.to_string e)
      end;
      true in
  
    ignore (Glib.Io.add_watch [`IN] cb (Glib.Io.channel_of_descr fd)) in

  let handle_pprz_message = fun (msg_id, values) ->
    let msg = Tele_Pprz.message_of_id msg_id in
    let s = String.concat " " (List.map snd values) in
    Ivy.send (sprintf "%s %s" msg.Pprz.name s) in

  let port = Sys.argv.(2) in

  listen_tty handle_pprz_message port;

  let loop = Glib.Main.create true in
  while Glib.Main.is_running loop do ignore (Glib.Main.iteration true) done

