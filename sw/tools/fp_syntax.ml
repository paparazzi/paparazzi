(*
   $Id$

   Syntax of flight plan expressions
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

let c_var_of_ident = fun x -> "_var_" ^ x

let rec sprint_expression = function
    Ident i when i.[0] = '$' -> sprintf "%s" (c_var_of_ident (String.sub i 1 (String.length i - 1)))
  | Ident i -> sprintf "%s" i
  | Int i -> sprintf "%d" i
  | Float i -> sprintf "%f" i
  | CallOperator (op, [e1;e2]) ->
      sprintf "(" ^ sprint_expression e1 ^ op ^ sprint_expression e2 ^ ")"
  | CallOperator (op, [e1]) ->
      sprintf "%s(%s)" op (sprint_expression e1)
  | CallOperator (_,_) -> failwith "Operator should be binary or unary"
  | Call (i, es) ->
      let ses = List.map sprint_expression es in
      sprintf "%s(" i ^ String.concat "," ses ^ ")"
  | Index (i,e) -> sprintf "%s[" i ^ sprint_expression e ^ "]"

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
  "estimator_flight_time";
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
  "nav_utm_east0"; "nav_utm_north0"; "nav_utm_zone0"; "climb_level_gaz"; "gps_lost"

]

exception Unknown_ident of string
exception Unknown_operator of string
exception Unknown_function of string

let rec check_expression = fun e ->
  match e with
    Ident i when i.[0] = '$' -> ()
  | Ident i ->
      if not (List.mem i variables) then
	raise (Unknown_ident i)
  | Int _  | Float _ | CallOperator _ -> ()
  | Call (i, es) ->
      if not (List.mem i functions) then
	raise (Unknown_function i);
      List.iter check_expression es
  | Index (i,e) ->
      if not (List.mem i variables) then
	raise (Unknown_ident i);
      check_expression e
