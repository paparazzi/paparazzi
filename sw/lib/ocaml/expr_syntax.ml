(*
 * Syntax of expressions à la C
 *
 * Copyright (C) 2003-2010 Antoine Drouin, Pascal Brisset, ENAC
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

type ident = string

type operator = string
type expression =
  | Ident of ident
  | Int of int
  | Float of float
  | Call of ident * (expression list)
  | CallOperator of ident * (expression list)
  | Index of ident * expression
  | Field of ident * ident
  | Deref of expression * ident

let c_var_of_ident = fun x -> "_var_" ^ x

let sprint = fun ?call_assoc expr ->
  let n, l = match call_assoc with
  | None -> None, []
  | Some (n, l) -> Some n, l
  in
  let rec eval = function
    | Ident i when i.[0] = '$' -> sprintf "%s" (c_var_of_ident (String.sub i 1 (String.length i - 1)))
    | Ident i -> sprintf "%s" i
    | Int i -> sprintf "%d" i
    | Float i -> sprintf "%f" i
    | CallOperator (op, [e1;e2]) ->
        sprintf "(%s%s%s)" (eval e1) op (eval e2)
    | CallOperator (op, [e1]) ->
        sprintf "%s(%s)" op (eval e1)
    | CallOperator (_,_) -> failwith "Operator should be binary or unary"
    | Call (i, [Ident s]) when Some i = n ->
        let index = try List.assoc s l with Not_found -> failwith (sprintf "Expr_syntax call_assoc not found: '%s'" s) in
        sprintf "%d" index
    | Call (i, es) ->
        let ses = List.map eval es in
        sprintf "%s(%s)" i (String.concat "," ses)
    | Index (i,e) -> sprintf "%s[%s]" i (eval e)
    | Field (i,f) -> sprintf "%s.%s" i f
    | Deref (e,f) -> sprintf "(%s)->%s" (eval e) f
  in
  eval expr

(* Valid functions : FIXME *)
let functions = [
  "Qdr";
  "And";
  "Or";
  "RcRoll";
  "RcEvent1";
  "RcEvent2";
  "RadOfDeg"]

(* Valid identifiers : FIXME *)
let variables = [
  "launch";
  "estimator_z";
  "autopilot_flight_time";
  "estimator_hspeed_mod";
  "estimator_theta";
  "circle_count";
  "vsupply";
  "stage_time";
  "stage_time_ds";
  "block_time";
  "SECURITY_ALT";
  "ground_alt"; "GROUND_ALT";
  "TRUE";
  "FALSE";
  "QFU";
  "gps_mode"; "gps_utm_east"; "gps_utm_north"; "gps_utm_zone";
  "nav_utm_east0"; "nav_utm_north0"; "nav_utm_zone0"; "cruise_throttle"; "gps_lost"

]

exception Unknown_ident of string
exception Unknown_operator of string
exception Unknown_function of string

let unexpected = fun kind x ->
  fprintf stderr "Warning: unexpected %s in expression: '%s' \n" kind x

let rec check_expression = fun e ->
  match e with
      Ident i when i.[0] = '$' -> ()
    | Ident i ->
      if not (List.mem i variables) then
        unexpected "ident" i
    | Int _  | Float _ | CallOperator _ -> ()
    | Call (i, es) ->
      if not (List.mem i functions) then
        unexpected "function" i;
      List.iter check_expression es
    | Index (i,e) ->
      if not (List.mem i variables) then
        unexpected "ident" i;
      check_expression e
    | Field (i, _field) ->
      if not (List.mem i variables) then
        unexpected "ident" i
    | Deref (e, _field) ->
      check_expression e
