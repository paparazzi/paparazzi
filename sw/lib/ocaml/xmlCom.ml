(*
 * XML parsing keeping everything (comments and declarations)
 *
 * Copyright (C) 2008, Cyril Allignol, Pascal Brisset
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

type state = A | B | C | D | D' | D'' | E

let children = function
Nethtml.Element (_tag, _params, children) -> children
  | _ -> invalid_arg "XmlCom.children"


(** Translate <tag .../> to <tag ...></tag> and parse  *)
let parse_file = fun file ->
  ignore (Xml.parse_file file);
  let buff = Buffer.create 5
  and lookup = Buffer.create 5
  and name = Buffer.create 5
  and chin = open_in file in
  let rec automata = fun state ->
    let char = input_char chin in
    let mem_and_continue = fun state ->
      Buffer.add_char lookup char;
      automata state
    and copy_and_continue = fun state ->
      Buffer.add_string buff (Buffer.contents lookup);
      Buffer.clear lookup;
      Buffer.add_char buff char;
      automata state
    and replace_and_continue = fun state ->
      Buffer.add_string buff "></";
      Buffer.add_string buff (Buffer.contents name);
      Buffer.add_char buff '>';
      Buffer.clear name; Buffer.clear lookup;
      automata state
    in
    match state, char with
        A, '<' -> copy_and_continue B
      | A, _   -> copy_and_continue A

      | B, '!' -> copy_and_continue A
      | B, (' ' | '\t' | '\n') -> copy_and_continue B
      | B, _   -> Buffer.add_char name char; copy_and_continue C

      | C, (' ' | '\t' | '\n') -> copy_and_continue D
      | C, '>' -> Buffer.clear name; copy_and_continue A
      | C, '/' -> mem_and_continue E
      | C, _   -> Buffer.add_char name char; copy_and_continue C

      | D, '/' -> mem_and_continue E
      | D, '>' -> Buffer.clear name; copy_and_continue A
      | D, '"' -> copy_and_continue D'
    | D, _   -> copy_and_continue D

    (* Inside a quoted string *)
    | D', '"' -> copy_and_continue D
      | D', '\\' -> automata D''
      | D', _  -> copy_and_continue D'

    (* Inside a quoted string, just after a \ (backslash) *)
      | D'', '"' -> Buffer.add_string buff "&quot;"; automata D'
    | D'', _  -> Buffer.add_char buff '\\'; copy_and_continue D'

    | E, '>' -> replace_and_continue A
    | E, _   -> copy_and_continue D
  in
  try
    ignore (automata A); failwith "Fichier sans fin"
  with End_of_file ->
    let buff = Buffer.contents buff in
    let lexbuf = Lexing.from_string buff in
    Nethtml.Element ("root", [], Nethtml.parse_document ~return_comments:true ~return_declarations:true lexbuf)


(** Translate <tag ...></tag> to <tag .../> *)
let ugly2nice = fun file ->
  let buff = Buffer.create 5
  and lookup = Buffer.create 5
  and chin = open_in file in
  let rec automata = fun state ->
    let char = input_char chin in
    let mem_and_continue = fun state ->
      Buffer.add_char lookup char;
      automata state
    and copy_and_continue = fun state ->
      Buffer.add_string buff (Buffer.contents lookup);
      Buffer.clear lookup;
      Buffer.add_char buff char;
      automata state
    and replace_and_continue = fun state ->
      Buffer.add_string buff "/>";
      Buffer.clear lookup;
      automata state
    in
    match state, char with
      A, '>' -> mem_and_continue B
    | A, _   -> copy_and_continue A
    | B, '<' -> mem_and_continue C
    | B, _   -> copy_and_continue A
    | C, '/' -> mem_and_continue D
    | C, _   -> copy_and_continue A
    | D, '>' -> replace_and_continue A
    | D, _   -> mem_and_continue D
    | _      -> failwith "This shouldn't occur..."
  in
  try
    ignore (automata A); failwith "Fichier sans fin"
  with End_of_file ->
    let s = Buffer.contents buff
    and chout = open_out file in
    Printf.fprintf chout "%s" s;
    close_out chout


(** Write XML and translate elements with no children *)
let to_file = fun xml filename ->
  let chout = new Netchannels.output_channel (open_out filename) in
  Nethtml.write ~dtd:[] chout (children xml);
  chout#close_out ();
  ugly2nice filename
