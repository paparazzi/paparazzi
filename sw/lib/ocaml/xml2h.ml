(*
 * $Id$
 *
 * XML preprocessing tools
 *
 * Copyright (C) 2003 Pascal Brisset, Antoine Drouin
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

open Printf

exception Error of string

let nl = print_newline

let define = fun n x ->
  printf "#define %s %s\n" n x

let define_string = fun n x ->
  define n ("\""^x^"\"")


let xml_error s = failwith ("Bad XML tag: "^s^ " expected")


let sprint_float_array = fun l ->
  let rec loop = function
      [] -> "}"
    | [x] -> x ^ "}"
    | x::xs -> x ^","^ loop xs in
  "{" ^ loop l


let start_and_begin = fun xml_file h_name ->
  let xml = Xml.parse_file xml_file in

  printf "/* This file has been generated from %s */\n" xml_file;
  printf "/* Please DO NOT EDIT */\n\n";

  printf "#ifndef %s\n" h_name;
  define h_name "";
  nl ();
  xml

let start_and_begin_c = fun xml_file name ->
  let xml = Xml.parse_file xml_file in
  printf "/* This file has been generated from %s */\n" xml_file;
  printf "/* Please DO NOT EDIT */\n\n";
  printf "#include \"%s.h\"\n" name;
  nl ();
  xml

let begin_c_out = fun xml_file name out ->
  fprintf out "/* This file has been generated from %s */\n" xml_file;
  fprintf out "/* Please DO NOT EDIT */\n\n";
  fprintf out "#include \"%s.h\"\n\n" name

let finish = fun h_name ->
  printf "\n#endif // %s\n" h_name

let warning s =
  Printf.fprintf stderr "##################################################\n";
  Printf.fprintf stderr "\n %s\n" s;
  Printf.fprintf stderr "\n##################################################\n"

