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

open Printf

(** simple boolean expressions *)
type bool_expr =
  | Any
  | Var of string
  | Not of bool_expr
  | And of bool_expr * bool_expr
  | Or of bool_expr * bool_expr

(** evaluate a boolean expression for a given value *)
let rec eval_bool v = function
  | Any -> true
  | Var x -> v = x
  | Not e -> not (eval_bool v e)
  | And (e1, e2) -> eval_bool v e1 && eval_bool v e2
  | Or (e1, e2) -> eval_bool v e1 || eval_bool v e2

(** pretty print boolean expression *)
let print_bool = fun v e ->
  let rec print_b v = function
    | Any -> eprintf "Any "
    | Var x -> eprintf "Var ( %s =? %s ) " x v
    | Not e -> eprintf "Not ( "; (print_b v e); eprintf ") "
    | And (e1, e2) -> eprintf "And ( "; print_b v e1; print_b v e2; eprintf ") "
    | Or (e1, e2) -> eprintf "Or ( "; print_b v e1; print_b v e2; eprintf ") "
  in
  print_b v e; eprintf "\n"

(** pretty print boolean expression *)
let sprint_bool = fun v e ->
  let rec print_b s v = function
    | Any -> sprintf "%sAny " s
    | Var x -> sprintf "%sVar ( %s =? %s ) " s x v
    | Not e -> let s = sprintf "%sNot ( " s in
               let s = print_b s v e in
               sprintf "%s) " s
    | And (e1, e2) -> let s = sprintf "%sAnd ( " s in
                      let s = print_b s v e1 in
                      let s =  print_b s v e2 in
                      sprintf "%s) " s
    | Or (e1, e2) -> let s = sprintf "%sOr ( " s in
                     let s = print_b s v e1 in
                     let s = print_b s v e2 in
                     sprintf "%s) " s
  in
  print_b "" v e


(** remove all duplicated elements of a list *)
let singletonize = fun ?(compare = compare) l ->
  let rec loop = fun l ->
    match l with
    | [] | [_] -> l
    | x::((x'::_) as xs) -> if compare x x' = 0 then loop xs else x::loop xs in
  loop (List.sort compare l)

(** union of two lists *)
let union = fun l1 l2 -> singletonize (l1 @ l2)

(** union of a list of list *)
let union_of_lists = fun l -> singletonize (List.flatten l)

(** [targets_of_string]
 * Returns the targets expression of a string
 *)
let targets_of_string =
  let rec expr_of_targets op = function
    | [] -> Any
    | [e] -> Var e
    | l::ls -> op (Var l) (expr_of_targets op ls)
  in
  let pipe = Str.regexp "|" in
  fun targets ->
    match targets with 
    | None -> Any
    | Some t ->
        if String.length t > 0 && String.get t 0 = '!' then
          Not (expr_of_targets (fun x y -> Or(x,y)) (Str.split pipe (String.sub t 1 ((String.length t) - 1))))
        else
          expr_of_targets (fun x y -> Or(x,y)) (Str.split pipe t)


(** [test_targets target targets]
 * Test if [target] is allowed [targets]
 * Return true if target is allowed, false if target is not in list or rejected (prefixed by !) *)
let test_targets = fun target targets ->
  eval_bool target targets

