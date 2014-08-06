(*
 * Log player
 *
 * Copyright (C) 2004-2009 ENAC, Pascal Brisset, Antoine Drouin
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


let set_title_and_bounds = fun window (adj:GData.adjustment) xml_file ->
  let (start, end_) = Play_core.get_log_bounds () in
  adj#set_bounds ~lower:start ~upper:end_ ();
  adj#set_value start;
  window#set_title (Filename.basename xml_file)

let open_log = fun window adj () ->
  Play_core.stop ();
  ignore (Log_file.chooser ~callback:(fun name -> Play_core.load_log name; set_title_and_bounds window adj name) ())


let () =
  let (serial_port, adj, speed) = Play_core.init () in

  let icon = GdkPixbuf.from_file Env.icon_rep_file in
  let window = GWindow.dialog ~icon ~title:"Replay" ~width:300 () in
  let quit = fun () -> GMain.Main.quit (); exit 0 in
  ignore (window#connect#destroy ~callback:quit);

  let menubar = GMenu.menu_bar ~packing:window#vbox#pack () in
  let factory = new GMenu.factory menubar in
  let accel_group = factory#accel_group in
  let file_menu = factory#add_submenu "File" in
  let file_menu_fact = new GMenu.factory file_menu ~accel_group in

  let timescale = GRange.scale `HORIZONTAL ~adjustment:adj ~packing:window#vbox#pack () in

  ignore (file_menu_fact#add_item "Open Log" ~key:GdkKeysyms._O ~callback:(open_log window adj));
  ignore (file_menu_fact#add_item "Play" ~key:GdkKeysyms._X ~callback:(fun () -> timescale#misc#set_sensitive false; Play_core.play serial_port adj speed));
  ignore (file_menu_fact#add_item "Stop" ~key:GdkKeysyms._S ~callback:(fun () -> timescale#misc#set_sensitive true; Play_core.stop ()));
  ignore (file_menu_fact#add_item "Quit" ~key:GdkKeysyms._Q ~callback:quit);

  window#add_accel_group accel_group;
  window#show ();

  if !Play_core.file_to_load <> "" then begin
    set_title_and_bounds window adj !Play_core.file_to_load;
    Play_core.play serial_port adj speed
  end;

  Play_core.main ()
