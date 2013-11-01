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

let icon_fixedwing_template =
  [
    [|  0.; -6.;  0.; 14.|];
    [| -9.;  0.;  9.;  0.|];
    [| -4.; 10.;  4.; 10.|]
  ]

class widget = fun ?(color="red") ?(icon_template=icon_fixedwing_template) (group:GnoCanvas.group) ->
  let new_line width color points =
    GnoCanvas.line ~fill_color:color ~props:[`WIDTH_PIXELS width; `CAP_STYLE `ROUND] ~points:points group in
  let icon_bg =
    List.map (fun points -> new_line 6 "black" points) icon_template in
  let icon =
    List.map (fun points -> new_line 4 color points) icon_template in
object(self)
  method set_color color =
    List.iter (fun segment -> segment#set [`FILL_COLOR color]) icon
  method set_bg_color color =
    List.iter (fun segment -> segment#set [`FILL_COLOR color]) icon_bg
end