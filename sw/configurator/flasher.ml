(*
 * $Id$
 *
 * Micro-controllers uploading
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

type mcu = Fbw | Ap | Modem

let string_of = function
    Fbw -> "Fly by wire"
  | Ap -> "Autopilot"
  | Modem -> "Modem"


let switch = fun mcu ->
  Dialog.create Widget.default_toplevel "Paparazzi upload" (Printf.sprintf "Please connect programmer board to %s\n" (string_of mcu)) ["OK"; "Cancel"] ()
      
let warn =
  let dont_bother_me = Textvariable.create () in
  fun continue ->
    if Textvariable.get dont_bother_me = "1" then
      continue ()
    else
      let t = Toplevel.create Widget.default_toplevel in
      Wm.title_set t "Paparazzi warning";
      
      Grab.set t;
      
      let destroy = fun () -> Tk.destroy t; continue () in
      
      let erase_AP = fun () ->
	if switch Ap = 0 then
	  let command = Printf.sprintf "cd %s; make erase" Env.ap_dir in
	  Console.exec command;
	  destroy () in
      
      let l = Label.create ~text:"Warning: You are about to program the fbw microcontroller. It is possible only if the AP microcontroller is erased." t
      and b = Button.create ~text:"Erase AP first" ~command:erase_AP t
      and b' = Button.create ~text:"Ok (AP erased)" ~command:destroy t
      and b'' = Button.create ~text:"Cancel" ~command:(fun () -> Tk.destroy t)  t
      and x = Checkbutton.create ~text:"Stop warning me about that" ~variable:dont_bother_me t in
      
      Tk.pack [l];
      Tk.pack [b;b';b''] ~side:`Left;
      Tk.pack [x]

let make = fun mcu path target ->
  let do_it = fun () ->
    if switch mcu = 0 then
      let command = Printf.sprintf "cd %s; make %s" path target in
      Console.exec command in
  
  if mcu = Fbw && target <> "erase" then warn do_it else do_it ()
  
