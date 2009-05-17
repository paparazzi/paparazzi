(*
 * $Id$
 *
 * Utilities
 *
 * Copyright (C) 2004 CENA/ENAC, Yann Le Fablec
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



let open_compress file = 
  if Filename.check_suffix file "gz" or Filename.check_suffix file "Z" or
	Filename.check_suffix file "zip" or Filename.check_suffix file "ZIP" then
    Unix.open_process_in ("gunzip -c "^file)
  else if Filename.check_suffix file "bz2"  then
    Unix.open_process_in ("bunzip2 -c "^file)
  else Pervasives.open_in file


let extensions = ["";".gz";".Z";".bz2";".zip";".ZIP"]
let find_file = fun path file ->
  let rec loop_path = function
      [] -> raise Not_found
    | p::ps ->
	let rec loop_ext = function
	    [] -> loop_path ps
	  | ext::es ->
	      let f = Filename.concat p file ^ ext in
	      if Sys.file_exists f then f else loop_ext es in
	loop_ext extensions in
  loop_path path

let regexp_plus = Str.regexp "\\+" 
let affine_transform = fun format ->
  match Str.split regexp_plus format with
    [a;b] -> float_of_string a, float_of_string b
  | [a] -> float_of_string a, 0.
  | _ -> 1., 0.
