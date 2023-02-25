(*
 * A Label with a contrasting outline
 *
 * Copyright (C) 2013 Piotr Esden-Tempski <piotr@esden.net>
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

(*
 * This module creates labels with outlines by creating 9
 * overlapping labels slightly offset from eachother. Where the 8
 * labels in the background have a different color from last center one.
 *)

let label_offset_matrix =
  [
   (* X    Y *)
    ( 0., -1.); (* N *)
    ( 0.,  1.); (* S *)
    ( 1.,  0.); (* E *)
    (-1.,  0.); (* W *)
    ( 1., -1.); (* NE *)
    ( 1.,  1.); (* SE *)
    (-1.,  1.); (* SW *)
    (-1., -1.); (* NW *)
    ( 0.,  0.); (* Z *)
  ]

class widget = fun ?(name = "Noname") ?(size = 500) ?(bg_color = "black") ?(color = "white") x y (group:GnoCanvas.group) ->
  let new_text offset =
    GnoCanvas.text group ~props:[`TEXT name;
                                 `X (x +. (fst offset)); `Y (y +. (snd offset));
                                 `ANCHOR `SW;
                                 `FILL_COLOR (if offset = (0., 0.) then color else bg_color)] in
  let labels = List.map new_text label_offset_matrix in
object(self)
  method set_name s = List.iter (fun label -> label#set [`TEXT s]) labels
  method set_x x = List.iter2 (fun label offset -> label#set [`X (x +. (fst offset))])
                              labels label_offset_matrix
  method set_y y = List.iter2 (fun label offset -> label#set [`Y (y +. (snd offset))])
                              labels label_offset_matrix
  method affine_absolute a = List.iter (fun label -> label#affine_absolute a) labels
end

