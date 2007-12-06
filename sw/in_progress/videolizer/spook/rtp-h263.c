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

struct rtp_h263 {
	unsigned char *d;
	int len;
	int init_done;
	int ts_incr;
	unsigned int timestamp;
};

static int h263_process_frame( struct frame *f, void *d )
{
	struct rtp_h263 *out = (struct rtp_h263 *)d;

	/* Discard the first two bytes, which are both 0x00 */
	out->d = f->d + 2;
	out->len = f->length - 2;
	out->timestamp += out->ts_incr;
	return out->init_done;
}

static int h263_get_sdp( char *dest, int len, int payload, int port, void *d )
{
	return snprintf( dest, len, "m=video %d RTP/AVP %d\r\na=rtpmap:%d H263-1998/90000\r\n", port, payload, payload );
}

static int h263_send( struct rtp_endpoint *ep, void *d )
{
	struct rtp_h263 *out = (struct rtp_h263 *)d;
	int i, plen;
	struct iovec v[3];
	unsigned char vhdr[2] = { 0x04, 0x00 }; /* Set P bit */

	v[1].iov_base = vhdr;
	v[1].iov_len = 2;

	for( i = 0; i < out->len; i += plen )
	{
		plen = out->len - i;
		if( plen > ep->max_data_size ) plen = ep->max_data_size;
		v[2].iov_base = out->d + i;
		v[2].iov_len = plen;
		if( send_rtp_packet( ep, v, 3, out->timestamp,
						plen + i == out->len ) < 0 )
			return -1;
		vhdr[0] = 0; /* clear P bit */
	}
	return 0;
}

struct rtp_media *new_rtp_media_h263_stream( struct stream *stream )
{
	struct rtp_h263 *out;
	int fincr, fbase;

	stream->get_framerate( stream, &fincr, &fbase );
	out = (struct rtp_h263 *)malloc( sizeof( struct rtp_h263 ) );
	out->init_done = 1;
	out->timestamp = 0;
	out->ts_incr = 90000 * fincr / fbase;

	return new_rtp_media( h263_get_sdp, NULL,
					h263_process_frame, h263_send, out );
}
