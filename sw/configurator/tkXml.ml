(*
 *  $Id$
 *
 * Binding of a widget to an XML file
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

let create_display_frame = fun f xml appli ->
  let tf = Frame.create f
  and af = Frame.create f in

  let l = Label.create ~text:"Name:" tf
  and name = Entry.create ~textvariable:(VarXml.attrib xml "name") tf in

  let destroy = fun () ->
    Tk.destroy tf; 
    Tk.destroy af;
    Grab.release f in
  
  let save = fun file ->
    Console.write (Printf.sprintf "Saving to \"%s\"\n" file);
    let s = Xml.to_string_fmt (VarXml.to_xml xml) in
    let f = open_out file in
    output_string f s;
    close_out f;
    destroy () in

  let save_button = Button.create ~text:"Save Config" ~command:(fun _ -> Env.select_one_file save) tf 
  and cancel_button = Button.create ~text:"Cancel" ~command:destroy tf  in
  
  Tk.pack [l] ~side:`Left;
  Tk.pack [name] ~side:`Left;
  Tk.pack [save_button; cancel_button] ~side:`Left;
  Tk.pack ~anchor:`N [tf];

  Grab.set f;

  Tk.pack [af];

  appli af xml



let create = fun top appli ->
  let f = Frame.create top in
  let load = fun file ->
    Console.write (Printf.sprintf "Reading from \"%s\"\n" file);
    
    create_display_frame f (VarXml.of_xml (Xml.parse_file file)) appli in
  let load_button = Button.create ~text:"Load Config" ~command:(fun _ -> Env.select_one_file load) top in

  Tk.pack [load_button];
  Tk.pack [f]
  

let create_section = fun f xml ->
  Tk.grid ~column:0 ~row:0 [Label.create ~text:"Name" f];
  Tk.grid ~column:1 ~row:0 [Label.create ~text:"Value" f];

  let r = ref 0 in
  List.iter
    (fun def ->
      incr r;
      Tk.grid ~column:0 ~row:!r [Label.create ~textvariable:(VarXml.attrib def "name") f];
      Tk.grid ~column:1 ~row:!r [Entry.create ~width:4 ~textvariable:(VarXml.attrib def "value") f])
    (VarXml.children xml)
    
