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
#include <errno.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <event.h>
#include <log.h>
#include <pmsg.h>
#include <rtp.h>

/* md5.h must be enclosed in quotes to avoid including OS-bundled headers */
#include "md5.h"

struct digest_auth_info {
	char realm[128];
	char uri[512];
	char nonce[128];
	char opaque[128];
	char username[128];
	char response[33];
};

static char digest_secret[17];
static int secret_created = 0;

static void create_digest_secret(void)
{
	random_id( digest_secret, 16 );
	secret_created = 1;
}

static void md5_hash( char **v, int count, char *hash )
{
	struct MD5Context md5;
	int i;
	unsigned char bin[16];

	MD5Init( &md5 );
	for( i = 0; i < count; ++i )
	{
		if( i > 0 ) MD5Update( &md5, ":", 1 );
		MD5Update( &md5, v[i], strlen( v[i] ) );
	}
	MD5Final( bin, &md5 );
	for( i = 0; i < 16; ++i ) sprintf( hash + (i<<1), "%02x", bin[i] );
	hash[32] = 0;
}

/*
 * Nonce format:
 *
 * Bytes  0- 3: date of nonce creation in seconds since epoch
 * Bytes  4-15: random bytes
 * Bytes 16-31: MD5( bytes[0..15] + ":" + digest secret )
 */
static void create_nonce( struct digest_auth_info *auth )
{
	unsigned char token[16];
	char *v[2] = { auth->nonce, digest_secret };
	struct timeval now;
	int i;

	if( ! secret_created ) create_digest_secret();

	gettimeofday( &now, NULL );
	PUT_32( token, now.tv_sec );
	random_bytes( token + 4, sizeof( token ) - 4 );
	for( i = 0; i < 16; ++i )
		sprintf( auth->nonce + (i<<1), "%02x", token[i] );
	auth->nonce[32] = 0;
	md5_hash( v, 2, auth->nonce + 32 );
}

static int parse_auth_header( char *header, struct digest_auth_info *auth )
{
	char *n, *v;
	int len;

	if( strncasecmp( header, "digest ", 7 ) ) return -1;
	n = header + 7;
	while( *n )
	{
		/* Look for the '=' after the directive name */
		for( v = n; *v != '='; ++v ) if( ! *v ) return -1;
		/* Step past the '=' */
		if( ! *(++v) ) return -1;
		/* See if the value is quoted */
		if( *v == '"' )
		{
			++v;
			/* Count characters until we find another quote */
			for( len = 0; v[len] != '"'; ++len )
				if( ! v[len] ) return -1;
		} else
		{
			/* Count characters until we find a comma, space, NUL */
			for( len = 0; v[len] != ',' && v[len] != ' ' &&
					v[len] != 0; ++len );
		}
		/* Do the appropriate thing for each directive that we handle */
		if( ! strncasecmp( n, "realm=", 6 ) )
		{
			if( len >= sizeof( auth->realm ) ) return -1;
			strncpy( auth->realm, v, len );
			auth->realm[len] = 0;
		} else if( ! strncasecmp( n, "uri=", 4 ) )
		{
			if( len >= sizeof( auth->uri ) ) return -1;
			strncpy( auth->uri, v, len );
			auth->uri[len] = 0;
		} else if( ! strncasecmp( n, "nonce=", 6 ) )
		{
			if( len >= sizeof( auth->nonce ) ) return -1;
			strncpy( auth->nonce, v, len );
			auth->nonce[len] = 0;
		} else if( ! strncasecmp( n, "opaque=", 7 ) )
		{
			if( len >= sizeof( auth->opaque ) ) return -1;
			strncpy( auth->opaque, v, len );
			auth->opaque[len] = 0;
		} else if( ! strncasecmp( n, "username=", 9 ) )
		{
			if( len >= sizeof( auth->username ) ) return -1;
			strncpy( auth->username, v, len );
			auth->username[len] = 0;
		} else if( ! strncasecmp( n, "response=", 9 ) )
		{
			if( len >= sizeof( auth->response ) ) return -1;
			strncpy( auth->response, v, len );
			auth->response[len] = 0;
		} else if( ! strncasecmp( n, "algorithm=", 10 ) )
		{
			/* If this is included, just make sure it is "MD5" */
			if( strncasecmp( v, "md5", 3 ) ) return -1;
		}
		/* Advance past the value */
		n = v + len;
		/* Advance past any trailing quotes, commas or spaces */
		while( *n == '"' || *n == ',' || *n == ' ' ) ++n;
	}
	return 0;
}

static void create_response( char *response, struct digest_auth_info *auth,
				char *method, char *password )
{
	char *elem[3], ha1[33], ha2[33];

	elem[0] = auth->username;
	elem[1] = auth->realm;
	elem[2] = password;
	md5_hash( elem, 3, ha1 );
	elem[0] = method;
	elem[1] = auth->uri;
	md5_hash( elem, 2, ha2 );
	elem[0] = ha1;
	elem[1] = auth->nonce;
	elem[2] = ha2;
	md5_hash( elem, 3, response );
}

static unsigned int get_hex_u32( char *hex )
{
	int i;
	unsigned int val = 0;

	for( i = 0; i < 8; ++i )
	{
		if( hex[i] >= '0' && hex[i] <= '9' )
			val = ( val << 4 ) | ( hex[i] - 0 );
		else if( hex[i] >= 'A' && hex[i] <= 'F' )
			val = ( val << 4 ) | ( hex[i] - 'A' + 10 );
		else if( hex[i] >= 'a' && hex[i] <= 'f' )
			val = ( val << 4 ) | ( hex[i] - 'a' + 10 );
		else break;
	}
	return val;
}

int check_digest_response( struct pmsg *msg, char *realm,
				char *username, char *password )
{
	char *hdr, expected[33], token[33];
	struct digest_auth_info auth;
	char *v[2] = { token, digest_secret };
	struct timeval now;

	memset( &auth, 0, sizeof( auth ) );

	if( ! ( hdr = get_header( msg, "authorization" ) ) ) return -1;
	if( parse_auth_header( hdr, &auth ) < 0 )
	{
		spook_log( SL_VERBOSE,
		    "digest-auth: unable to parse www-authenticate header" );
		return -1;
	}

	/* Case-sensitive realm (should just be parroted back by the client) */
	if( strcmp( auth.realm, realm ) )
	{
		spook_log( SL_VERBOSE,
			"digest-auth: realm \"%s\" is not correct",
			auth.realm );
		return -1;
	}
	/* Case-insensitive usernames */
	if( strcasecmp( auth.username, username ) )
	{
		spook_log( SL_VERBOSE,
			"digest-auth: username \"%s\" is not correct",
			auth.username );
		return -1;
	}

	/* The client may or may not have included the digest-uri directive */
	if( ! auth.uri[0] )
	{
		if( strlen( msg->sl.req.uri ) >= sizeof( auth.uri ) )
		{
			spook_log( SL_WARN,
				"URI is too long for digest-auth!" );
			return -1;
		}
		strcpy( auth.uri, msg->sl.req.uri );
	}

	/* Figure out what the response should be */
	create_response( expected, &auth, msg->sl.req.method, password );
	if( strcasecmp( auth.response, expected ) )
	{
		spook_log( SL_VERBOSE, "digest-auth: incorrect password" );
		return -1;
	}

	/* From this point on, it appears that the client knows the correct
	 * username and password, it's just a matter of whether the nonce
	 * was generated by us and has not yet expired.  If the nonce is
	 * invalid, we can use the "stale=true" directive in the 401 so
	 * the client can retry authentication with the same username and
	 * password, if it still has it. */

	if( ! secret_created || strlen( auth.nonce ) != 64 )
	{
		spook_log( SL_VERBOSE, "digest-auth: this is not our nonce!" );
		return 0;
	}

	/* Check that the nonce validates against our secret */
	strncpy( token, auth.nonce, 32 );
	token[32] = 0;
	md5_hash( v, 2, expected );
	if( strcmp( auth.nonce + 32, expected ) )
	{
		spook_log( SL_VERBOSE, "digest-auth: this is not our nonce!" );
		return 0;
	}

	/* Check that the nonce is not more than 15 seconds old */
	gettimeofday( &now, NULL );
	if( now.tv_sec > get_hex_u32( token ) + 15 )
	{
		spook_log( SL_VERBOSE,
			"digest-auth: nonce is more than 15 seconds old" );
		return 0;
	}

	spook_log( SL_VERBOSE, "digest-auth authentication succeeded" );
	return 1;
}

int add_digest_challenge( struct pmsg *msg, char *realm, int stale )
{
	struct digest_auth_info auth;

	strcpy( auth.realm, realm );
	create_nonce( &auth );

	/* LIVE.COM expects the challenge in exactly this format */
	return add_header_printf( msg, "WWW-Authenticate",
			"Digest realm=\"%s\", nonce=\"%s\"%s",
			auth.realm, auth.nonce,
			stale ? ", stale=true" : "" );
}
