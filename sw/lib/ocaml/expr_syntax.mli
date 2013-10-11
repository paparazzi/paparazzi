(*
 * Syntax of expressions à la C
 *
 * Copyright (C) 2003-2010 CENA/ENAC, Pascal Brisset, Antoine Drouin
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

type ident = string
type operator = string
type expression =
  | Ident of ident
  | Int of int
  | Float of float
  | Call of ident * expression list
  | CallOperator of ident * expression list
  | Index of ident * expression
  | Field of ident * ident
  | Deref of expression * ident

val c_var_of_ident : ident -> string
(** Encapsulate a user ident into a C variable *)

val sprint : ?call_assoc:(ident * (ident * int) list) -> expression -> string
(** call_assoc: call an optional association list function *)

exception Unknown_ident of string
exception Unknown_operator of string
exception Unknown_function of string

val check_expression : expression -> unit
(** May raise [Unknown_ident], [Unknown_operator] or [Unknown_function]
   exceptions *)
