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
#include <sys/time.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>

#include <event.h>
#include <log.h>
#include <frame.h>
#include <stream.h>
#include <outputs.h>
#include <rtp.h>
#include <rtp_media.h>
#include <conf_parse.h>

static struct session *sess_list = NULL;

struct rtp_media *new_rtp_media( rtp_media_get_sdp_func get_sdp,
	rtp_media_get_payload_func get_payload, rtp_media_frame_func frame,
	rtp_media_send_func send, void *private )
{
	struct rtp_media *m;

	if( ! ( m = (struct rtp_media *)malloc( sizeof( *m ) ) ) )
		return NULL;
	m->get_sdp = get_sdp;
	m->get_payload = get_payload;
	m->frame = frame;
	m->send = send;
	m->private = private;
	return m;
}

void del_session( struct session *sess )
{
	if( sess->next ) sess->next->prev = sess->prev;
	if( sess->prev ) sess->prev->next = sess->next;
	else sess_list = sess->next;
	if( sess->control_close ) sess->control_close( sess );
	free( sess );
}

struct session *new_session(void)
{
	struct session *sess;
	int i;

	sess = (struct session *)malloc( sizeof( struct session ) );
	sess->next = sess_list;
	sess->prev = NULL;
	if( sess->next ) sess->next->prev = sess;
	sess_list = sess;
	sess->get_sdp = NULL;
	sess->setup = NULL;
	sess->play = NULL;
	sess->pause = NULL;
	sess->private = NULL;
	sess->control_private = NULL;
	sess->control_close = NULL;
	gettimeofday( &sess->open_time, NULL );
	strcpy( sess->addr, "(unconnected)" );
	for( i = 0; i < MAX_TRACKS; ++i ) sess->ep[i] = NULL;
	return sess;
}

int print_session_list( char *s, int maxlen )
{
	struct session *sess;
	int i = 0;

	for( sess = sess_list; sess; sess = sess->next )
		i += sprintf( s + i, "%s %ld\n",
					sess->addr, sess->open_time.tv_sec );
	return i;
}
