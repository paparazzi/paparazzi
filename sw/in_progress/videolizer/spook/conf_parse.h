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

#define TOKEN_NUM	1
#define TOKEN_STR	2

struct token {
	int type;
	union {
		int num;
		char str[256];
	} v;
};

struct statement {
	char *directive;
	int (*process)( int num_tokens, struct token *tokens, void *d );
	int min_args;
	int max_args;
	int types[10];
};

/* conf_parse.c */
void register_config_context( char *type, char *name,
				void *(*start_block)( void ),
				int (*end_block)( void *d ),
				struct statement *s );

/* conf_scan.l */
int start_conf_read( char *filename );
int get_next_token( struct token *tok, int *line );
