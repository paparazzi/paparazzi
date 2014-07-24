/*
 * Grammar à la C
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
 */
%{
open Expr_syntax
%}
%token <int> INT
%token <float> FLOAT
%token <string> IDENT
%token EOF
%token DOT COMMA SEMICOLON LP RP LC RC LB RB DEREF AND COLON OR
%token EQ GT ASSIGN GEQ NOT
%token PLUS MINUS
%token MULT DIV MOD EXPO

%left AND OR	/* lowest precedence */
%left EQ GT ASSIGN GEQ
%left PLUS MINUS
%left MULT DIV MOD EXPO
%nonassoc NOT
%nonassoc UMINUS
%left DEREF /* highest precedence */

%start expression	/* the entry point */
%type <Expr_syntax.expression> expression

%%

expression:
    expression GT expression { CallOperator (">",[$1;$3]) }
  | expression GEQ expression { CallOperator (">=",[$1;$3]) }
  | expression EQ expression { CallOperator ("==",[$1;$3]) }
  | expression AND expression { CallOperator ("&&",[$1;$3]) }
  | expression OR expression { CallOperator ("||",[$1;$3]) }
  | expression PLUS expression { CallOperator ("+",[$1;$3]) }
  | expression MINUS expression { CallOperator ("-",[$1;$3]) }
  | expression MULT expression { CallOperator ("*",[$1;$3]) }
  | expression DIV expression { CallOperator ("/",[$1;$3]) }
  | expression MOD expression { CallOperator ("%",[$1;$3]) }
  | expression EXPO expression { CallOperator ("**",[$1;$3]) }
  | MINUS expression %prec UMINUS { CallOperator ("-",[$2]) }
  | NOT expression { CallOperator ("!",[$2]) }
  | INT { Int $1 }
  | FLOAT { Float $1 }
  | IDENT { Ident $1 }
  | IDENT DOT IDENT { Field ($1,$3) }
  | expression DEREF IDENT { Deref($1, $3) } 
  | IDENT LP Args RP { Call ($1, $3) }
  | LP expression RP { $2 }
  | IDENT LB expression RB { Index ($1, $3) }
;

Args: { [] }
  | expression NextArgs { $1::$2 }
;

NextArgs: { [] }
  | COMMA expression NextArgs { $2::$3 }
;
