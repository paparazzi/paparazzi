/* $Id$ */
%{
open Fp_syntax
%}
%token <int> INT
%token <float> FLOAT
%token <string> IDENT
%token EOF
%token COMMA SEMICOLON LP RP LC RC LB RB AND COLON
%token EQ GT ASSIGN GEQ NOT
%token PLUS MINUS
%token MULT DIV


%left EQ GT ASSIGN GEQ	/* lowest precedence */
%left PLUS MINUS
%left MULT DIV
%nonassoc NOT
%nonassoc UMINUS	/* highest precedence */

%start expression	/* the entry point */
%type <Fp_syntax.expression> expression

%%

expression:
    expression GT expression { CallOperator (">",[$1;$3]) }
  | expression GEQ expression { CallOperator (">=",[$1;$3]) }
  | expression EQ expression { CallOperator ("==",[$1;$3]) }
  | expression AND expression { CallOperator ("&&",[$1;$3]) }
  | expression PLUS expression { CallOperator ("+",[$1;$3]) }
  | expression MINUS expression { CallOperator ("-",[$1;$3]) }
  | expression MULT expression { CallOperator ("*",[$1;$3]) }
  | expression DIV expression { CallOperator ("/",[$1;$3]) }
  | MINUS expression %prec UMINUS { CallOperator ("-",[$2]) }
  | NOT expression { CallOperator ("!",[$2]) }
  | INT { Int $1 }
  | FLOAT { Float $1 }
  | IDENT { Ident $1 }
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
