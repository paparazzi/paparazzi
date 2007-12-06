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

static const int frame_rate_tab[][2] = {
	{ 0, 0 },
	{ 1001, 24000 },
	{ 1, 24 },
	{ 1, 25 },
	{ 1001, 30000 },
	{ 1, 30 },
	{ 1, 50 },
	{ 1001, 60000 },
	{ 1, 60 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 },
	{ 0, 0 }
};

struct rtp_mpv {
	struct {
		unsigned char *d;
		int len;
	} blk[48];
	int blk_count;
	int temporal_reference;
	int picture_type;
	int vectors;
	unsigned char vsh[4096];
	int vsh_len;
	int init_done;
	int ts_incr;
	unsigned int timestamp;
};

static int find_next_code( unsigned char *d, int len )
{
	int i;

	/* Start at 1 to ignore the code sitting at d[0] */
	for( i = 1; i < len - 2; ++i )
		if( d[i] == 0x00 && d[i+1] == 0x00 && d[i+2] == 0x01 )
			return i;
	return 0;
}

static unsigned int get_field( unsigned char *d, int bits, int *offset )
{
	unsigned int v = 0;
	int i;

	for( i = 0; i < bits; )
	{
		if( bits - i >= 8 && *offset % 8 == 0 )
		{
			v <<= 8;
			v |= d[*offset/8];
			i += 8;
			*offset += 8;
		} else
		{
			v <<= 1;
			v |= ( d[*offset/8] >> ( 7 - *offset % 8 ) ) & 1;
			++i;
			++(*offset);
		}
	}
	return v;
}

static void parse_video_sequence_header( struct rtp_mpv *out,
				unsigned char *d, int len )
{
	int off = 32 + 12 + 12 + 4, frc;

	frc = get_field( d, 4, &off );
	out->ts_incr = 90000 * frame_rate_tab[frc][0] / frame_rate_tab[frc][1];
	out->init_done = 1;
}

static void parse_picture_header( struct rtp_mpv *out,
				unsigned char *d, int len )
{
	int off = 32;

	if( ! out->init_done ) return;

	out->timestamp += out->ts_incr;
	out->temporal_reference = get_field( d, 10, &off );
	out->picture_type = get_field( d, 3, &off );
	out->vectors = 0;
	off += 16;
	if( out->picture_type == 2 || out->picture_type == 3 )
	{
		out->vectors = get_field( d, 1, &off ) << 3; /* FFV */
		out->vectors |= len > 8 ? get_field( d, 3, &off ) : /* FFC */
					( get_field( d, 2, &off ) << 1 );
		if( out->picture_type == 3 && len > 8 )
		{
			out->vectors |= get_field( d, 1, &off ) << 7; /* FBV */
			out->vectors |= get_field( d, 3, &off ) << 4; /* BFC */
		}
	}
}

static int mpv_process_frame( struct frame *f, void *d )
{
	struct rtp_mpv *out = (struct rtp_mpv *)d;
	int flen, start, in_picture = 0, have_vsh = 0;
	unsigned int start_code;

	out->blk_count = 0;
	for( start = 0; start < f->length; start += flen )
	{
		if( out->blk_count == 48 )
		{
			spook_log( SL_WARN,
				"rtp-mpv: too many elements to send" );
			break;
		}
		flen = find_next_code( f->d + start, f->length - start );
		if( flen == 0 ) flen = f->length - start;

		start_code = GET_32( f->d + start );
		if( start_code <= 0x000001AF || start_code == 0x000001B8 )
			in_picture = 1;

		if( in_picture )
		{
			if( start_code == 0x00000100 )
				parse_picture_header( out, f->d + start, flen );
			out->blk[out->blk_count].d = f->d + start;
			out->blk[out->blk_count].len = flen;
			++out->blk_count;
		} else /* we have not seen a GOP or picture start code yet */
		{
			if( start_code == 0x000001B3 )
			{
				out->vsh_len = 0;
				have_vsh = 1;
				parse_video_sequence_header( out,
						f->d + start, flen );
			}
			if( have_vsh )
			{
				memcpy( out->vsh + out->vsh_len,
						f->d + start, flen );
				out->vsh_len += flen;
			}
		}
	}
	return out->init_done;
}

static int mpv_get_sdp( char *dest, int len, int payload, int port, void *d )
{
	return snprintf( dest, len, "m=video %d RTP/AVP 32\r\n", port );
}

static int mpv_get_payload( int payload, void *d )
{
	return 32;
}

static int mpv_send( struct rtp_endpoint *ep, void *d )
{
	struct rtp_mpv *out = (struct rtp_mpv *)d;
	int i, j, space, off, min_space;
	struct iovec v[48];
	unsigned char vhdr[4];

	/* MPEG video header always follows the RTP header */
	PUT_16( vhdr, out->temporal_reference );
	vhdr[2] = out->picture_type; /* will be different for each frame */
	vhdr[3] = out->vectors;
	v[1].iov_base = vhdr;
	v[1].iov_len = 4;

	i = 0;
	j = 2;
	space = ep->max_data_size - 4;

	/* If this is an I frame, insert the saved Video Sequence Header */
	if( out->picture_type == 1 )
	{
		vhdr[2] |= 0x20; /* Sequence-header-present bit */

		/* add the block to the frame */
		v[j].iov_base = out->vsh;
		v[j].iov_len = out->vsh_len;

		space -= v[j].iov_len;
		++j;
	}

	for( i = 0; i < out->blk_count; ++i )
	{
		/* We can fragment slices, but after the initial fragment,
		 * all the remaining fragments must be put into their own
		 * packets.  This means fragmentation only makes sense if
		 * the slice is larger than our MTU. */
		if( out->blk[i].len > ep->max_data_size - 4 ) min_space = 4;
		else min_space = out->blk[i].len;

		/* If we don't have enough space for this entire block, or if
		 * we're fragmenting and we don't have enough space for the
		 * start code, first send out the previous blocks. */
		if( space < min_space )
		{
			if( send_rtp_packet( ep, v, j, out->timestamp, 0 ) < 0 )
				return -1;
			j = 2;
			vhdr[2] = out->picture_type;
			space = ep->max_data_size - 4;
		}

		/* add the block to the frame */
		v[j].iov_base = out->blk[i].d;
		v[j].iov_len = out->blk[i].len;

		/* if this is a slice, set the Beginning-of-slice bit */
		if( out->blk[i].d[3] > 0 && out->blk[i].d[3] <= 0xAF )
			vhdr[2] |= 0x10;

		/* if the entire block fit, go on to the next block */
		if( v[j].iov_len <= space )
		{
			/* if this is a slice, set the End-of-slice bit */
			if( out->blk[i].d[3] > 0 && out->blk[i].d[3] <= 0xAF )
				vhdr[2] |= 0x08;
			space -= v[j].iov_len;
			++j;
			continue;
		}

		/* block did not fit, so send the first fragment */
		v[j].iov_len = space;
		/* the last byte of the packet is not the end of a slice, so
		 * clear the End-of-slice bit */
		vhdr[2] &= ~0x08;
		if( send_rtp_packet( ep, v, j + 1, out->timestamp, 0 ) < 0 )
			return -1;

		/* send all remaining fragments by themselves */
		for( off = space; off < out->blk[i].len;
				off += v[2].iov_len )
		{
			vhdr[2] = out->picture_type;
			v[2].iov_base = out->blk[i].d + off;
			v[2].iov_len = out->blk[i].len - off;
			if( v[2].iov_len > ep->max_data_size - 4 )
				v[2].iov_len = ep->max_data_size - 4;
			else
				vhdr[2] |= 0x08; /* End-of-slice bit */
			if( send_rtp_packet( ep, v, 3, out->timestamp,
					( vhdr[2] & 0x08 ) &&
					( ( i + 1 ) == out->blk_count ) ) < 0 )
				return -1;
		}
		j = 2;
		vhdr[2] = out->picture_type;
		space = ep->max_data_size - 4;
	}

	/* send any unsent blocks */
	if( j > 2 && send_rtp_packet( ep, v, j, out->timestamp, 1 ) < 0 )
		return -1;

	return 0;
}

struct rtp_media *new_rtp_media_mpv(void)
{
	struct rtp_mpv *out;

	out = (struct rtp_mpv *)malloc( sizeof( struct rtp_mpv ) );
	out->init_done = 0;
	out->timestamp = 0;

	return new_rtp_media( mpv_get_sdp, mpv_get_payload, mpv_process_frame,
			mpv_send, out );
}
