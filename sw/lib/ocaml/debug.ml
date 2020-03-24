(*
 * Debugging facilities
 *
 * Copyright (C) 2004 CENA/ENAC, Pascal Brisset, Antoine Drouin
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


let level = ref (try Sys.getenv "PPRZ_DEBUG" with Not_found -> "")
let log = ref stderr
let call lev f =
  assert( (* assert permet au compilo de tout virer avec l'option -noassert *)
    if (String.contains !level '*' || String.contains !level lev)
    then begin
      f !log;
      flush !log
    end;
    true)

let trace lev s = call lev (fun f -> Printf.fprintf f "%s\n" s)

let xprint = fun s ->
  let n = String.length s in
  let a = Bytes.make (3*n) ' ' in
  for i = 0 to n - 1 do
    let x = Printf.sprintf "%02x" (Char.code s.[i]) in
    Bytes.set a (3*i) x.[0];
    Bytes.set a (3*i+1) x.[1]
  done;
  Bytes.to_string a
