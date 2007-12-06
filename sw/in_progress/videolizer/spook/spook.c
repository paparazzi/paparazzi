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
#include <signal.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <pthread.h>
#include <errno.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <inputs.h>
#include <encoders.h>
#include <filters.h>
#include <outputs.h>
#include <rtp.h>
#include <conf_parse.h>
#include <config.h>

int read_config_file( char *config_file );
int spook_log_init( int min );

static int init_random(void)
{
	int fd;
	unsigned int seed;

	if( ( fd = open( "/dev/urandom", O_RDONLY ) ) < 0 )
	{
		spook_log( SL_ERR, "unable to open /dev/urandom: %s",
				strerror( errno ) );
		return -1;
	}
	if( read( fd, &seed, sizeof( seed ) ) < 0 )
	{
		spook_log( SL_ERR, "unable to read from /dev/urandom: %s",
				strerror( errno ) );
		return -1;
	}
	close( fd );
	srandom( seed );
	return 0;
}

void random_bytes( unsigned char *dest, int len )
{
	int i;

	for( i = 0; i < len; ++i )
		dest[i] = random() & 0xff;
}

void random_id( unsigned char *dest, int len )
{
	int i;

	for( i = 0; i < len / 2; ++i )
		sprintf( dest + i * 2, "%02X",
				(unsigned int)( random() & 0xff ) );
	dest[len] = 0;
}

int main( int argc, char **argv )
{
	int i;
	enum { DB_NONE, DB_FOREGROUND, DB_DEBUG } debug_mode = DB_NONE;
	char *config_file = "spook.conf";

	while( ( i = getopt( argc, argv, "Dfc:" ) ) != -1 )
	{
		switch( i )
		{
		case 'D':
			debug_mode = DB_DEBUG;
			break;
		case 'f':
			if( debug_mode == DB_NONE )
				debug_mode = DB_FOREGROUND;
			break;
		case 'c':
			config_file = optarg;
			break;
		case '?':
		case ':':
			return 1;
			break;
		}
	}

	switch( debug_mode )
	{
	case DB_NONE:
		spook_log_init( SL_INFO );
		break;
	case DB_FOREGROUND:
		spook_log_init( SL_VERBOSE );
		break;
	case DB_DEBUG:
		spook_log_init( SL_DEBUG );
		break;
	}

	if( init_random() < 0 ) return 1;

	access_log_init();

	oss_init();
#ifdef SPOOK_INPUT_V4L
	v4l_init();
#endif
#ifdef SPOOK_INPUT_V4L2
	v4l2_init();
#endif
#ifdef SPOOK_INPUT_DC1394
	dc1394_init();
#endif
#ifdef SPOOK_INPUT_VDIG
	vdig_init();
#endif
#ifdef SPOOK_ENCODER_JPEG
	jpeg_init();
#endif
	framedrop_init();
	alaw_init();
#ifdef SPOOK_ENCODER_MPEG4
	mpeg4_init();
	mpeg4_dec_init();
#endif
	mp2_init();
	live_init();
	http_init();
	control_listen();

	if( read_config_file( config_file ) < 0 ) return 1;

	event_loop( 0 );

	return 0;
}

/********************* GLOBAL CONFIGURATION DIRECTIVES ********************/

int config_frameheap( int num_tokens, struct token *tokens, void *d )
{
	int size, count;

	signal( SIGPIPE, SIG_IGN );

	count = tokens[1].v.num;
	if( num_tokens == 3 ) size = tokens[2].v.num;
	else size = 352*288*3;

	spook_log( SL_DEBUG, "frame size is %d", size );

	if( count < 10 )
	{
		spook_log( SL_ERR, "frame heap of %d frames is too small, use at least 10", count );
		return -1;
	}

	init_frame_heap( size, count );

	return 0;
}
