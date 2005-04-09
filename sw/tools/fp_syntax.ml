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
  | Index of ident * expression

let c_var_of_ident = fun x -> "_var_" ^ x

(* Valid unary and binary opetarors *)
let binary_operators = ["+"; ">"; "-"; "*"]
let unary_operators = ["!"; "-"]

let is_binary = fun op -> List.mem op binary_operators
let is_unary = fun op -> List.mem op unary_operators

let rec sprint_expression = function
    Ident i when i.[0] = '$' -> sprintf "%s" (c_var_of_ident (String.sub i 1 (String.length i - 1)))
  | Ident i -> sprintf "%s" i
  | Int i -> sprintf "%d" i
  | Float i -> sprintf "%f" i
  | Call (op, [e1;e2]) when is_binary op ->
      sprintf "(" ^ sprint_expression e1 ^ op ^ sprint_expression e2 ^ ")"
  | Call (op, [e1]) when is_unary op ->
      sprintf "%s(%s)" op (sprint_expression e1)
  | Call (i, es) ->
      let ses = List.map sprint_expression es in
      sprintf "%s(" i ^ String.concat "," ses ^ ")"
  | Index (i,e) -> sprintf "%s[" i ^ sprint_expression e ^ "]"

(* Valid functions *)
let functions = [
  "Qdr";
  "And";
  "Or";
  "RcEvent1";
  "RcEvent2"] @ binary_operators @ unary_operators

(* Valid identifiers *)
let variables = [
  "launch";
  "estimator_z";
  "estimator_flight_time";
  "stage_time";
  "block_time";
  "SECURITY_ALT";
  "GROUND_ALT";
  "TRUE";
  "QFU"
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
  | Int _  | Float _ -> ()
  | Call (i, es) ->
      if not (List.mem i functions) then
	raise (Unknown_function i);
      List.iter check_expression es
  | Index (i,e) ->
      if not (List.mem i variables) then
	raise (Unknown_ident i);
      check_expression e
