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
  | '$'?['a'-'z' '_' 'A'-'Z'] (['a'-'z' 'A'-'Z' '_' '.' '0'-'9']*) { IDENT (Lexing.lexeme lexbuf) }
  | '\''[^'\'']+'\'' { let s = Lexing.lexeme lexbuf in IDENT (String.sub s 1 (String.length s - 2)) }
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
  | "%" { MOD }
  | ">=" { GEQ }
  | "+" { PLUS }
  | "=" { ASSIGN }
  | "-" { MINUS }
  | "*" { MULT }
  | "/" { DIV }
  | "!" { NOT }
  | eof { EOF }

