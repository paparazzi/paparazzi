(*
 *  $Id$
 *
 * Configuration graphic interface 
 *  
 * Copyright (C) 2004 Pascal Brisset, Antoine Drouin
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

let exit = fun () ->
  if Dialog.create Widget.default_toplevel "Configurator quit" "Really quit ?\n" ["OK"; "Cancel"] () = 0 then 
    exit 0

let _ =
  let top = Widget.default_toplevel in
  Wm.title_set top "The Paparazzi Configurator";

  let sheets = Frame.create top in

  Welcome.create_sheet sheets;
  Hardware.create_sheet sheets;
  Radio.create_sheet sheets;
  Airframe.create_sheet sheets;
(***
  Flightplan.create_sheet sheets;
  Upload.create_sheet sheets;
  Simulator.create_sheet sheets;
  Monitor.create_sheet sheets;
  Logalizer.create_sheet sheets;
***)

  let quit_button = Button.create ~relief:`Sunken ~text:"Quit" ~command:exit sheets
  and (n, _) = Grid.size sheets in
  Tk.grid ~column:n ~row:0 [quit_button];

  let console = Console.create top in

  Tk.pack [sheets];
  Tk.pack [console];

  Tk.mainLoop()
