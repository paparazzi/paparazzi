(*
 *  $Id$
 *
 * Attitude control configuration
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

let create_sheet = fun sheets xml ->
  let f = Frame.create sheets in
  Notebook.create_sheet sheets "Attitude" f;


  let label = fun l -> Widget.forget_type (Label.create ~text:l f) in

  let empty = Widget.dummy in

  let entry _ = Widget.forget_type (Label.create f) in

  let array =
    [| [| entry ();  label "Min";  label "Max"; label "Gain"|];
       [| label "Roll"; entry (); entry (); entry ()|];
       [| label "Pitch"; entry (); entry (); entry ()|];
       [| label "Throttle"; entry (); entry (); entry ()|] |] in

  Array.iteri 
    (fun i -> Array.iteri (fun j w -> Tk.grid ~row:i ~column:j [w]))
    array
