(*
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

let pi = 3.14159265358979323846;;

let open_compress file =
  if Filename.check_suffix file "gz" || Filename.check_suffix file "Z" ||
    Filename.check_suffix file "zip" || Filename.check_suffix file "ZIP" then
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

let regexp_plus_less = Str.regexp "[+-]"
let affine_transform = fun format ->
  (* Split after removing blank spaces *)
  let split = Str.full_split regexp_plus_less (Str.global_replace (Str.regexp "[ \t]+") "" format) in
  let first_sign = match List.hd split with
      Str.Text _ | Str.Delim "+" -> 1.
    | Str.Delim "-" -> -. 1.
    | _ -> 0.
  in
  let second_sign = match split with
      [_; Str.Delim "+"; _] | [_; _; Str.Delim "+"; _] -> 1.
    | [_; Str.Delim "-"; _]
    | [_; Str.Delim "+"; Str.Delim "-"; _]
    | [_; _; Str.Delim "-"; _]
    | [_; _; Str.Delim "+"; Str.Delim "-"; _] -> -1.
    | _ -> 0.
  in
  match split with
      [Str.Text a; _; Str.Text b]
    | [_; Str.Text a; _; Str.Text b]
    | [Str.Text a; _; _; Str.Text b]
    | [_; Str.Text a; _; _; Str.Text b] -> first_sign *. float_of_string a, second_sign *. float_of_string b
    | [Str.Text a] | [_; Str.Text a] -> first_sign *. float_of_string a, 0.
    | _ -> 1., 0.

(* Box-Muller transform to generate a normal distribution from a uniform one
   http://en.wikipedia.org/wiki/Normal_distribution *)
let normal = fun mu sigma ->
  let u1 = Random.float 1.
  and u2 = Random.float 1. in
  mu +. sigma *. sqrt (-2. *. log u1) *. cos (2. *. pi *. u2)

let make_1st_order_noise_generator = fun ?(init = 0.) k sigma ->
  let x = ref init in
  fun () ->
    x := k *. !x +. normal 0. sigma;
    !x

let shifter = fun n default ->
  let a = Array.make n default
  and i = ref 0 in
  fun new_value ->
    let old_value = a.(!i) in
    a.(!i) <- new_value;
    i := (!i + 1) mod n;
    old_value
