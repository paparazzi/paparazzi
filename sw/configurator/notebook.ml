(*
 *  $Id$
 *
 * Facility to create tabs (thanx to JB)
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


let eval l = 
  let token x = Protocol.TkToken x in
  Protocol.tkEval (Array.map token (Array.of_list l))

(* Float weights bug for grid *)
let set_weight orient w index weight =
  ignore (eval ["grid"; orient ^ "configure"; Widget.name w;
		string_of_int index; "-weight"; string_of_int weight])

let set_column_weight w = set_weight "column" w
let set_row_weight    w = set_weight "row" w

let conf_relief = fun b value ->
  eval [Widget.name b; "config"; "-relief"; value]

let create_sheet w name child =
  let lower b = conf_relief b "sunken"
  and upper b = conf_relief b "flat" in
  let b = Button.create ~relief:`Sunken ~text:name w in
  let cmd () =
    let (n, _) = Grid.size w in
    let slaves = Grid.slaves ~row:1 w in
    if slaves <> [] then Grid.forget slaves;
    List.iter (fun w -> ignore (lower w)) (Grid.slaves ~row:0 w);
    ignore (upper b);
    Tk.grid ~column:0 ~row:1 ~columnspan:n ~padx:20 (*** ~pady:20 ***) ~sticky:"news" [child] in
  Button.configure ~borderwidth:1 ~command:cmd b;
  let (n, _) = Grid.size w in
  Tk.grid ~column:n ~row:0 ~sticky:"news" [b];
  set_column_weight w n 1;
  if n = 0 then (cmd (); set_row_weight w 1 1)
  else Grid.configure ~columnspan:(succ n) (Grid.slaves ~row:1 w)

  
  
