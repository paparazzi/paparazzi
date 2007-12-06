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

#include <sys/types.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <encoders.h>
#include <filters.h>
#include <conf_parse.h>
#include <global_config.h> /* Don't include this in other files */

struct config_context {
	struct config_context *next;
	char type[256];
	char name[256];
	void *(*start_block)( void );
	int (*end_block)( void *d );
	struct statement *statements;
};

/* Create a global config context pointing to the global_statements array
 * defined in global_config.h */
static struct config_context global_context = {
	NULL, {}, {}, NULL, NULL, global_statements
};

void register_config_context( char *type, char *name,
				void *(*start_block)( void ),
				int (*end_block)( void *d ),
				struct statement *s )
{
	struct config_context *cc;

	for( cc = &global_context; cc->next; cc = cc->next );
	cc->next = (struct config_context *)
				malloc( sizeof( struct config_context ) );
	cc = cc->next;
	cc->next = NULL;
	strcpy( cc->type, type );
	strcpy( cc->name, name );
	cc->start_block = start_block;
	cc->end_block = end_block;
	cc->statements = s;
}

static int process_statement( struct config_context *cc, void *context_data,
				struct token *t, int num_tokens, int line )
{
	struct statement *s;
	char *directive = t[0].v.str;
	int num_args = num_tokens - 1, i;
	int seen = 0;
	struct token *args = t + 1;

	if( t[0].type != TOKEN_STR )
	{
		spook_log( SL_ERR,
			"line %d: malformed configuration directive (unexpected numeral?)" );
		return -1;
	}

	for( s = cc->statements; s->directive; ++s )
		if( ! strcasecmp( s->directive, directive ) )
		{
			seen = 1;
			if( num_args < s->min_args || num_args > s->max_args )
				continue;
			for( i = 0; i < num_args; ++i )
				if( s->types[i] != args[i].type ) break;
			if( i < num_args ) continue;
			s->process( num_tokens, t, context_data );
			return 0;
		}

	if( seen )
		spook_log( SL_ERR,
			"line %d: wrong number/type of arguments for directive \"%s\"",
			line, directive );
	else
		spook_log( SL_ERR,
			"line %d: unknown configuration directive \"%s\"",
			line, directive );
	return -1;
}

int read_config_file( char *config_file )
{
	struct token t[10];
	int cur_token = 0, line;
	struct config_context *cc = &global_context;
	void *context_data = NULL;

	if( start_conf_read( config_file ) < 0 ) return -1;

	while( get_next_token( &t[cur_token], &line ) > 0 )
	{
		switch( t[cur_token].type )
		{
		case TOKEN_NUM:
		case TOKEN_STR:
			if( ++cur_token == 10 )
			{
				spook_log( SL_ERR,
					"line %d: max number of arguments exceeded",
					line );
				return -1;
			}
			break;
		case ';':
			if( process_statement( cc, context_data,
						t, cur_token, line ) < 0 )
				return -1;
			cur_token = 0;
			break;
		case '{':
			if( cur_token != 2 || t[0].type != TOKEN_STR || t[1].type != TOKEN_STR )
			{
				spook_log( SL_ERR,
					"line %d: blocks must begin with a type and module name",
					line );
				return -1;
			}
			if( cc != &global_context )
			{
				spook_log( SL_ERR,
					"line %d: unexpected '{' (missing '}'?)",
					line );
				return -1;
			}
			while( ( cc = cc->next ) &&
				( strcasecmp( cc->type, t[0].v.str ) != 0 ||
				  strcasecmp( cc->name, t[1].v.str ) != 0 ) );
			if( ! cc )
			{
				spook_log( SL_ERR,
					"line %d: unknown module %s %s", line,
						t[0].v.str, t[1].v.str );
				return -1;
			}
			if( cc->start_block &&
				  ( context_data = cc->start_block() ) == NULL )
				return -1;
			cur_token = 0;
			break;
		case '}':
			if( cur_token != 0 )
			{
				spook_log( SL_ERR,
					"line %d: last statement in block is missing a semicolon",
					line );
				return -1;
			}
			if( cc == &global_context )
			{
				spook_log( SL_ERR,
					"line %d: unbalanced }", line );
				return -1;
			}
			if( cc->end_block && cc->end_block( context_data ) < 0 )
				return -1;
			cur_token = 0;
			cc = &global_context;
			context_data = NULL;
			break;
		default:
			spook_log( SL_ERR,
				"internal parser error: unrecognized type %d",
				t[cur_token].type );
			return -1;
		}
	}

	return 0;
}
