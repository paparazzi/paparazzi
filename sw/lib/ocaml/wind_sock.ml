 (*
  * Wind sock
  *
  * Copyright (C) 2007 ENAC
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

 let flatten = fun s a ->
   let n = Array.length a in
   let b = Array.make (2*n) 0. in
   for i = 0 to n - 1 do
     let (x, y) = a.(i) in
     b.(2*i) <- float x *. s;
     b.(2*i+1) <- float y *. s
   done;
   b

 class item = fun ?(show = false) size_unit group ->
   let texture = `FILL_STIPPLE (Gdk.Bitmap.create_from_data ~width:2 ~height:2 "\002\001") in

   let group = GnoCanvas.group group in

   (* Text *)
   let t = GnoCanvas.text group ~props:[`TEXT "12.1"; `X 0.; `Y 0.; `ANCHOR `CENTER; `FILL_COLOR "black"] in

   (* Red left and right *)
   let props = [`FILL_COLOR "red"; texture] in
   let points = flatten size_unit [|(-6,4); (-2,3); (-2,-3); (-6,-4)|] in
   let _ = GnoCanvas.polygon group ~props ~points in
   let points = flatten size_unit [|(2,2); (6,1); (6,-1); (2,-2)|] in
   let _ = GnoCanvas.polygon group ~props ~points in

   (* White center *)
   let props = [`FILL_COLOR "white"] in
   let points = flatten size_unit [|(-2,3); (2,2); (2,-2); (-2,-3)|] in
   let _ = GnoCanvas.polygon group ~props ~points in

   (* contour *)
   let points = flatten size_unit [|(-6,4); (6,1); (6,-1); (-6,-4)|] in
   let props = [`OUTLINE_COLOR "white"; `WIDTH_PIXELS 1] in
   let contour = GnoCanvas.polygon group ~props ~points in

   object
     method item = group
     method label = t
     initializer
       t#raise_to_top ();
       if not show then group#hide ()
     method set_color = fun color -> contour#set [`OUTLINE_COLOR color]
   end
