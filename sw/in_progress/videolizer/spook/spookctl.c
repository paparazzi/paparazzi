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
#include <sys/un.h>

#include <event.h>

int fd;

static void stdin_read( struct event_info *ei, void *d )
{
	unsigned char c;

	if( read( 0, &c, 1 ) < 1 )
	{
		exit( 1 );
	}

	if( c != '\n' ) write( fd, &c, 1 );
}

static void sock_read( struct event_info *ei, void *d )
{
	unsigned char c[80];
	int ret;

	ret = read( fd, &c, 80 );
	if( ret <= 0 )
	{
		perror( "read" );
		exit( 1 );
	}

	write( 1, c, ret );
}

int main( int argc, char **argv )
{
	struct sockaddr_un addr;

	addr.sun_family = AF_UNIX;
	strcpy( addr.sun_path, "spook.sock" );

	if( ( fd = socket( PF_UNIX, SOCK_STREAM, 0 ) ) < 0 )
	{
		perror( "socket" );
		return 1;
	}
	if( connect( fd, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 )
	{
		perror( "connect" );
		return 1;
	}

	add_fd_event( fd, 0, 0, sock_read, NULL );
	add_fd_event( 0, 0, 0, stdin_read, NULL );

	write( 1, "connected\n", 10 );

	event_loop( 0 );

	return 0;
}
