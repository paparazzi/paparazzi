%{
/*
 * Copyright (C) 2004 Nathan Lutchansky <lutchann@litech.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <string.h>
#include <errno.h>

#include <log.h>
#include <conf_parse.h>

#define YY_FATAL_ERROR	scanner_error

static struct token *cur_token;
static int lineno;

void scanner_error( char *msg );
%}

%option noyywrap

%%

0[0-9]+			{
				cur_token->type = TOKEN_NUM;
				cur_token->v.num = strtol( yytext + 1, NULL, 8 );
				return cur_token->type;
			}

0x[0-9a-fA-F]+		{
				cur_token->type = TOKEN_NUM;
				cur_token->v.num = strtol( yytext + 2, NULL, 16 );
				return cur_token->type;
			}

[0-9]+			{
				cur_token->type = TOKEN_NUM;
				cur_token->v.num = strtol( yytext, NULL, 10 );
				return cur_token->type;
			}

[a-zA-Z/][a-zA-Z/0-9_\-\+@:\.%]* {
				cur_token->type = TOKEN_STR;
				strncpy( cur_token->v.str, yytext, yyleng );
				cur_token->v.str[yyleng] = 0;
				return cur_token->type;
			}

"\""[^"\n]*"\""		{
				cur_token->type = TOKEN_STR;
				strncpy( cur_token->v.str,
						yytext + 1, yyleng - 2 );
				cur_token->v.str[yyleng - 2] = 0;
				return cur_token->type;
			}

[ \t]+			/* whitespace */

"\n"			++lineno;

"#"[^\n]*		/* comment */

";"			|
"{"			|
"}"			{
				cur_token->type = *yytext;
				return cur_token->type;
			}

<<EOF>>			return 0;

.			return -1;

%%

void scanner_error( char *msg )
{
	spook_log( SL_ERR, "error scanning config file: %s", msg );
	exit( 1 );
}

int start_conf_read( char *filename )
{
	if( ! ( yyin = fopen( filename, "r" ) ) )
	{
		spook_log( SL_ERR, "unable to open %s: %s", filename,
				strerror( errno ) );
		return -1;
	}
	lineno = 1;
	return 0;
}

int get_next_token( struct token *tok, int *line )
{
	int ret;

	cur_token = tok;
	ret = yylex();
	*line = lineno;
	return ret;
}
