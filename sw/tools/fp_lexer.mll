(* 
   $Id$
*)
{
open Fp_parser
}
rule token = parse
    [' ' '\t' '\n'] { token lexbuf}
  | "/*"([^'*']|'*'[^'/'])*'*'*'/' { token lexbuf}
  | ['0'-'9']+ { INT (int_of_string (Lexing.lexeme lexbuf)) }
  | ['0'-'9']+'.'['0'-'9']* { FLOAT (float_of_string (Lexing.lexeme lexbuf)) }
  | '$'?['a'-'z' 'A'-'Z'] (['a'-'z' 'A'-'Z' '_' '.' '0'-'9']*) { IDENT (Lexing.lexeme lexbuf) }
  | ',' { COMMA }
  | ';' { SEMICOLON }
  | ':' { COLON }
  | '(' { LP }
  | ')' { RP }
  | '{' { LC }
  | '}' { RC }
  | '[' { LB } 
  | ']' { RB }
  | "==" { EQ }
  | "&&" { AND }
  | "||" { OR }
  | ">" { GT }
  | ">=" { GEQ }
  | "+" { PLUS }
  | "=" { ASSIGN }
  | "-" { MINUS }
  | "*" { MULT }
  | "/" { DIV }
  | "!" { NOT }
  | eof { EOF }

