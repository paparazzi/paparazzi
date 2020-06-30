(*
 * generic tools for modules
 *
 * Copyright (C) 2010 Gautier Hattenberger
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

(* simple boolean expressions *)
type bool_expr =
  | Any
  | Var of string
  | Not of bool_expr
  | And of bool_expr * bool_expr
  | Or of bool_expr * bool_expr

val print_bool : string -> bool_expr -> unit
val sprint_bool : string -> bool_expr -> string

(** remove all duplicated elements of a list *)
val singletonize : ?compare: ('a -> 'a -> int) -> 'a list -> 'a list

(** [targets_of_string] targets
 * Returns the targets expression of a string
 *)
val targets_of_string : string option -> bool_expr

(** [test_targets target targets]
 * Test if [target] is allowed [targets]
 * Return true if target is allowed, false if target is not in list or rejected (prefixed by !) *)
val test_targets : string -> bool_expr -> bool


