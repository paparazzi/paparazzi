(*
 *  $Id$
 *
 * Flight plans edition
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

open Latlong

let calibrate_xy = fun x0 y0 ->
  let scale = 2.5 in

  let print x y =
    let xl = x0 + truncate (float x *. scale)
    and yl = y0 + truncate (float y *. scale) in
    let wgs84 = Latlong.wgs84_of_lambertIIe xl yl in
    Console.write (Printf.sprintf "%d %d %f %f\n" x y ((Rad>>Deg)wgs84.Latlong.posn_lat)  ((Rad>>Deg)wgs84.Latlong.posn_long)) in
  
  Console.write (Printf.sprintf "Calibration for x0=%d y0=%d:\n--8<----------------------\n" x0 y0);
  
  print 0 0;
  print 1000 0;
  print 0 1000;
  Console.write (Printf.sprintf "--8<-----------------------\n")
  
let calibrate_ign_tile = fun filename ->
  Scanf.sscanf (Filename.basename filename) "F%3d_%3d" (fun x y ->
    let x0 = x * 10000
    and y0 = (267 - y) * 10000 in

    calibrate_xy x0 y0)

let int_of_tv = fun tv -> int_of_string (Textvariable.get tv)


let create_sheet = fun sheets ->
  let f = Frame.create sheets in
  Notebook.create_sheet sheets "Flight Plan" f;

  let b = Button.create ~text:"Calibrate IGN tile" ~command:(fun () -> Notebook.select_one_file ~filter:"*.png" calibrate_ign_tile) f in

  Tk.pack [b];

  let tvx = Textvariable.create ()
  and tvy = Textvariable.create () in

  let lx = Label.create ~text:"LambertIIe: x:" f
  and ex = Entry.create ~textvariable:tvx f
  and ly = Label.create ~text:"y:" f
  and ey = Entry.create ~textvariable:tvy f
  and c = Button.create ~text:"Calibrate" ~command:(fun () -> calibrate_xy (int_of_tv tvx) (int_of_tv tvy)) f in

  Tk.pack ~side:`Left [lx];
  Tk.pack ~side:`Left [ex];
  Tk.pack ~side:`Left [ly];
  Tk.pack ~side:`Left [ey];
  Tk.pack ~side:`Left [c]

  
