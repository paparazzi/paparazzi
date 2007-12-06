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

struct rtp_mpa {
	unsigned char *mpa_data;
	int mpa_len;
	int ts_incr;
	unsigned int timestamp;
};

static int mpa_get_sdp( char *dest, int len, int payload, int port, void *d )
{
	return snprintf( dest, len, "m=audio %d RTP/AVP 14\r\n", port );
}

static int mpa_get_payload( int payload, void *d )
{
	return 14;
}

static int mpa_process_frame( struct frame *f, void *d )
{
	struct rtp_mpa *out = (struct rtp_mpa *)d;
	const int samrate_tab[] = { 44100, 48000, 32000 };

	if( f )
	{
		out->timestamp += out->ts_incr;
		out->ts_incr = 90000 * 1152 / samrate_tab[(f->d[2] >> 2) & 0x3];
		out->mpa_data = f->d;
		out->mpa_len = f->length;
	} else
	{
		out->ts_incr = 0;
		out->mpa_len = 0;
	}

	return 1;
}

static int mpa_send( struct rtp_endpoint *ep, void *d )
{
	struct rtp_mpa *out = (struct rtp_mpa *)d;
	int i, plen;
	unsigned char mpahdr[4];
	struct iovec v[3];

	PUT_16( mpahdr, 0 );
	v[1].iov_base = mpahdr;
	v[1].iov_len = 4;

	for( i = 0; i < out->mpa_len; i += plen )
	{
		plen = out->mpa_len - i;
		if( plen > ep->max_data_size ) plen = ep->max_data_size;
		PUT_16( mpahdr + 2, i );
		v[2].iov_base = out->mpa_data + i;
		v[2].iov_len = plen;
		if( send_rtp_packet( ep, v, 3, out->timestamp,
						plen + i == out->mpa_len ) < 0 )
			return -1;
	}
	return 0;
}

struct rtp_media *new_rtp_media_mpa(void)
{
	struct rtp_mpa *out;

	out = (struct rtp_mpa *)malloc( sizeof( struct rtp_mpa ) );

	out->mpa_len = 0;
	out->ts_incr = 0;
	out->timestamp = 0;

	return new_rtp_media( mpa_get_sdp, mpa_get_payload, mpa_process_frame,
			mpa_send, out );
}
