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
#include <stdarg.h>
#include <fcntl.h>
#include <ctype.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>

#include <log.h>
#include <pmsg.h>
#include <config.h>

char *add_pmsg_string( struct pmsg *msg, char *s )
{
	int len = strlen( s ) + 1;
	char *p;

	if( msg->msg_len + len > msg->max_len ) return NULL;
	p = msg->msg + msg->msg_len;
	msg->msg_len += len;
	strcpy( p, s );
	return p;
}

static int inline delim( char *d, int len, char term1, char term2 )
{
	int s;

	for( s = 0; s < len; ++s )
		if( d[s] == term1 || d[s] == term2 ||
			       d[s] == '\r' || d[s] == '\n' )
			break;
	return s;
}

int parse_pmsg( struct pmsg *msg )
{
	int i = 0;
	unsigned char *d = msg->msg;
	int len = msg->msg_len;

	msg->header_count = 0;

	/* Was it all whitespace? */
	if( len - i < 4 ) return -1;

	/* Check for a slash in the first word */
	for( i = 0; i < len && d[i] != ' ' && d[i] != '/'; ++i );
	if( i == len ) return -1;

	/* Responses begin with "PROTO/" */
	if( d[i] == '/' )
	{
		msg->type = PMSG_RESP;
		/* The first word is the protocol name and version */
		msg->proto_id = d;
		i = delim( d, len, ' ', 0 );
		/* Check for a space following the version number */
		if( d[i] != ' ' ) return -1;
		d[i++] = 0;
		while( i < len && d[i] == ' ' ) ++i;
		/* The next word is the 3-digit response code, then a space */
		if( len - i < 4 || ! isdigit( d[i] ) || ! isdigit( d[i+1] )
				|| ! isdigit( d[i+2] ) || d[i+3] != ' ' )
			return -1;
		msg->sl.stat.code = atoi( d + i );
		i += 4;
		while( i < len && d[i] == ' ' ) ++i;
		/* The rest of the line is the textual response */
		msg->sl.stat.reason = d + i;
		i += delim( d + i, len - i, 0, 0 );
		d[i++] = 0;
	} else
	{
		msg->type = PMSG_REQ;
		/* The first word is the method */
		msg->sl.req.method = d;
		i = delim( d, len, ' ', 0 );
		/* Then a space */
		if( i >= len || d[i] != ' ' ) return -1;
		d[i++] = 0;
		while( i < len && d[i] == ' ' ) ++i;
		/* The second word is the URI */
		msg->sl.req.uri = d + i;
		i += delim( d + i, len - i, ' ', 0 );
		/* Then a space */
		if( i >= len || d[i] != ' ' ) return -1;
		d[i++] = 0;
		while( i < len && d[i] == ' ' ) ++i;
		/* The last word is the protocol name and version */
		msg->proto_id = d + i;
		i += delim( d + i, len - i, 0, 0 );
		d[i++] = 0;
	}
	/* Skip any trailing space */
	while( i < len && d[i] == ' ' ) ++i;
	/* Skip the \r if we didn't kill it already */
	if( i < len && d[i] == '\r' ) ++i;
	/* We should be at the end of the line now */
	if( i >= len || d[i++] != '\n' ) return -1;

	/* Now, parse all the header lines */
	for(;;)
	{
		/* There may be a \r here if we're at the end of the headers */
		if( i < len && d[i] == '\r' ) ++i;
		/* If there's no more data, we're done */
		if( i == len ) return len;
		/* If there's a newline, we're at the end of the headers */
		if( d[i] == '\n' ) return i + 1;
		/* XXX headers beginning with whitespace are continuations */
		if( d[i] == '\t' || d[i] == ' ' ) return -1;
		/* The first thing on the line is the header name */
		msg->fields[msg->header_count].name = d + i;
		/* The name ends with optional spaces then a colon */
		i += delim( d + i, len - i, ' ', ':' );
		if( i >= len ) return -1;
		/* If the optional spaces are present, skip them */
		if( d[i] == ' ' )
		{
			d[i++] = 0;
			while( i < len && d[i] == ' ' ) ++i;
		}
		/* Make sure the colon is present */
		if( i >= len || d[i] != ':' ) return -1;
		d[i++] = 0;
		/* Skip any whitespace after the colon */
		while( i < len && ( d[i] == ' ' || d[i] == '\t' ) ) ++i;
		/* Everything else on the line is the header data */
		msg->fields[msg->header_count].value = d + i;
		i += delim( d + i, len - i, 0, 0 );
		d[i++] = 0;
		/* We should be at the end of the line now */
		if( i >= len || d[i++] != '\n' ) return -1;
		++msg->header_count;
	}
}

char *get_header( struct pmsg *msg, char *name )
{
	int i;

	for( i = 0; i < msg->header_count; ++i )
		if( ! strcasecmp( msg->fields[i].name, name ) )
			return msg->fields[i].value;
	return NULL;
}

int add_header( struct pmsg *msg, char *name, char *value )
{
	if( msg->header_count == MAX_FIELDS ) return -1;
	/* Put the name and value back-to-back at the end of the message */
	msg->fields[msg->header_count].name = add_pmsg_string( msg, name );
	if( ! msg->fields[msg->header_count].name ) return -1;
	msg->fields[msg->header_count].value = add_pmsg_string( msg, value );
	if( ! msg->fields[msg->header_count].value ) return -1;
	++msg->header_count;
	return 0;
}

int add_header_printf( struct pmsg *msg, char *name, char *fmt, ... )
{
	va_list ap;
	int len;

	/* do the vsnprintf first to clean up the stack */
	va_start( ap, fmt );
	len = vsnprintf( msg->msg + msg->msg_len, msg->max_len - msg->msg_len,
			fmt, ap );
	va_end( ap );

	/* check for errors */
	if( msg->header_count == MAX_FIELDS ) return -1;
	/* vsnprintf will return the length of the formatted string
	 * regardless of length, although the actual output may be truncated */
	if( msg->msg_len + len >= msg->max_len ) return -1;
	msg->fields[msg->header_count].value = msg->msg + msg->msg_len;
	msg->msg_len += len + 1;
	/* add the name */
	msg->fields[msg->header_count].name = add_pmsg_string( msg, name );
	if( ! msg->fields[msg->header_count].name ) return -1;
	++msg->header_count;
	return 0;
}

int replace_header( struct pmsg *msg, char *name, char *value )
{
	int i;

	for( i = 0; i < msg->header_count; ++i )
		if( ! strcasecmp( msg->fields[i].name, name ) )
		{
			/* If we can't reuse the space from the original value,
			 * we have to allocate space for the new value at the
			 * end of the message */
			if( strlen( msg->fields[i].value ) < strlen( value ) )
			{
				char *p = add_pmsg_string( msg, value );
				if( ! p ) return -1;
				msg->fields[i].value = p;
			} else strcpy( msg->fields[i].value, value );
			return 0;
		}
	/* It doesn't exist, so we'll insert it as new */
	return add_header( msg, name, value );
}

int copy_headers( struct pmsg *dest, struct pmsg *src, char *name )
{
	int i, count = 0;

	for( i = 0; i < src->header_count; ++i )
		if( ! strcasecmp( src->fields[i].name, name ) )
		{
			add_header( dest, src->fields[i].name,
					src->fields[i].value );
			++count;
		}
	return count;
}

/* get_param() returns 1 for found, 0 for not found, -1 for error */
int get_param( char *value, char *tag, char *dest, int size )
{
	int taglen, i;
	char *c;

	if( ! value ) return -1;

	taglen = strlen( tag );

	for( c = value; c; c = strchr( c, ';' ) )
	{
		++c;
		if( ! strncasecmp( c, tag, taglen ) )
		{
			c += taglen;
			if( *c == 0 || *c == ';' ) /* Value-less tag */
			{
				if( dest && size > 0 ) *dest = 0;
				return 1;
			} else if( *c == '=' ) /* Tag has an associated value */
			{
				if( ! dest || size <= 0 ) return 1;
				++c;
				for( i = 0; *c && *c != ';' && i < size;
						++i, ++c )
					dest[i] = *c;
				if( i == size ) return -1;
				dest[i] = 0;
				return 1;
			} /* Otherwise, it's a false hit */
		}
	}
	return 0;
}

struct pmsg *new_pmsg( int size )
{
	struct pmsg *msg;
	void *v;

	if( ! ( v = malloc( sizeof( struct pmsg ) + size ) ) )
	{
		spook_log( SL_ERR, "unable to allocate memory for message" );
		return NULL;
	}
	msg = (struct pmsg *)v;
	msg->msg = v + sizeof( struct pmsg );
	msg->max_len = size;
	msg->msg_len = 0;
	msg->header_count = 0;
	msg->proto_id = NULL;
	return msg;
}

void free_pmsg( struct pmsg *msg )
{
	free( msg );
}
