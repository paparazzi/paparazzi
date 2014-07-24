(* 
 * Lexical tokens à la C
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
 *)
{
open Expr_parser
}
rule token = parse
    [' ' '\t' '\n'] { token lexbuf}
  | "/*"([^'*']|'*'[^'/'])*'*'*'/' { token lexbuf}
  | ['0'-'9']+ { INT (int_of_string (Lexing.lexeme lexbuf)) }
  | ['0'-'9']+'.'['0'-'9']* { FLOAT (float_of_string (Lexing.lexeme lexbuf)) }
  | '$'?['a'-'z' '_' 'A'-'Z'] (['a'-'z' 'A'-'Z' '_' '0'-'9']*) { IDENT (Lexing.lexeme lexbuf) }
  | '\''[^'\'']+'\'' { let s = Lexing.lexeme lexbuf in IDENT (String.sub s 1 (String.length s - 2)) }
  | ',' { COMMA }
  | '.' { DOT }
  | ';' { SEMICOLON }
  | ':' { COLON }
  | '(' { LP }
  | ')' { RP }
  | '{' { LC }
  | '}' { RC }
  | '[' { LB } 
  | ']' { RB }
  | "->" { DEREF }
  | "==" { EQ }
  | "&&" { AND }
  | "||" { OR }
  | ">" { GT }
  | "%" { MOD }
  | ">=" { GEQ }
  | "+" { PLUS }
  | "=" { ASSIGN }
  | "-" { MINUS }
  | "**" { EXPO }
  | "*" { MULT }
  | "/" { DIV }
  | "!" { NOT }
  | eof { EOF }

{
  let parse = fun s ->
    let lexbuf = Lexing.from_string s in
    try
      Expr_parser.expression token lexbuf
    with
      Failure("lexing: empty token") ->
	Printf.fprintf stderr "Lexing error in '%s': unexpected char: '%c' \n"
	  s (Lexing.lexeme_char lexbuf 0);
	exit 1
    | Parsing.Parse_error ->
	Printf.fprintf stderr "Parsing error in '%s', token '%s' ?\n"
	  s (Lexing.lexeme lexbuf);
	exit 1
}
