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
#include <sys/un.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <rtp.h>
#include <conf_parse.h>

struct ctl_sock {
	int fd;
	struct event *read_event;
};

struct listener {
	int fd;
};

static void drop_sock( struct ctl_sock *cs )
{
	spook_log( SL_DEBUG, "closed control connection" );
	remove_event( cs->read_event );
	close( cs->fd );
	free( cs );
}

static void do_read( struct event_info *ei, void *d )
{
	struct ctl_sock *cs = (struct ctl_sock *)d;
	unsigned char c, output[2048];

	if( read( cs->fd, &c, 1 ) < 1 )
	{
		drop_sock( cs );
		return;
	}


	switch( c )
	{
	case 'a':
		write( cs->fd, output,
			print_session_list( output, sizeof( output ) ) );
		break;
	case 'l':
		send_log_buffer( cs->fd );
		break;
	default:
		write( cs->fd, "unknown command\n", 16 );
		break;
	}
	write( cs->fd, ".\n", 2 );
}

static void do_accept( struct event_info *ei, void *d )
{
	struct listener *listener = (struct listener *)d;
	int fd, addrlen;
	struct sockaddr_un addr;
	struct ctl_sock *cs;

	addrlen = sizeof( addr );
	if( ( fd = accept( listener->fd, (struct sockaddr *)&addr, &addrlen ) ) < 0 )
	{
		spook_log( SL_WARN, "error accepting control connection: %s",
				strerror( errno ) );
		return;
	}
	spook_log( SL_DEBUG, "accepted control connection" );

	cs = (struct ctl_sock *)malloc( sizeof( struct ctl_sock ) );
	if( ! cs )
	{
		spook_log( SL_ERR, "out of memory on malloc ctl_sock" );
		close( fd );
		return;
	}
	cs->fd = fd;
	cs->read_event = add_fd_event( fd, 0, 0, do_read, cs );
}

int control_listen(void)
{
	struct sockaddr_un addr;
	struct listener *listener;
	int fd;

	addr.sun_family = AF_UNIX;
	strcpy( addr.sun_path, "spook.sock" );

	unlink( addr.sun_path );
	if( ( fd = socket( PF_UNIX, SOCK_STREAM, 0 ) ) < 0 )
	{
		spook_log( SL_ERR, "error creating control socket: %s",
				strerror( errno ) );
		return -1;
	}
	if( bind( fd, (struct sockaddr *)&addr, sizeof( addr ) ) < 0 )
	{
		spook_log( SL_ERR, "unable to bind control socket: %s",
				strerror( errno ) );
		close( fd );
		return -1;
	}
	if( listen( fd, 5 ) < 0 )
	{
		spook_log( SL_ERR,
			"error attempting to listen on control socket: %s",
			strerror( errno ) );
		close( fd );
		return -1;
	}

	listener = (struct listener *)malloc( sizeof( struct listener ) );
	listener->fd = fd;

	add_fd_event( fd, 0, 0, do_accept, listener );

	spook_log( SL_INFO, "listening on control socket %s", addr.sun_path );

	return 0;
}
