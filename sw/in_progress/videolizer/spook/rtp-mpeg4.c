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

/* MPEG4 Video ES format:
 * 
 * +--Included in SDP config line
 * |
 * | Visual Object Sequence
 * |   start code (0x000001B0), profile and level indication, stuff
 * |     Visual Object
 * |       start code (0x000001B5), visual object type, stuff
 * |     Video Object
 * |       start code (0x00000100-0x0000011F)
 * |     Video Object Layer
 * |       start code (0x00000120-0x0000012F), stuff to parse
 *           Group Video Object Plane*
 *             start code (0x000001B3), time codes, stuff
 *           Video Object Plane*
 *             start code (0x000001B6), vop coding type, modulo time base,
 *             vop time increment
 *     end code (0x000001B1)
 */

struct rtp_mpeg4 {
	struct iovec iov[24];
	int iov_count;
	unsigned char config[512];
	int config_len;
	int pali;
	char fmtp[600];
	int init_done;
	int ts_incr;
	int vop_time_increment_resolution;
	int vop_time_increment;
	int vtir_bitlen;
	unsigned int timestamp;
};

static int add_to_rtp_buf( struct rtp_mpeg4 *out, unsigned char *d, int len )
{
	if( out->iov_count == 24 )
	{
		spook_log( SL_WARN, "rtp-mpeg4: too many elements to send" );
		return -1;
	}
	out->iov[out->iov_count].iov_base = d;
	out->iov[out->iov_count].iov_len = len;
	++out->iov_count;
	return 0;
}

static int find_next_code( unsigned char *d, int len )
{
	int i;

	/* Start at 1 to ignore the code sitting at d[0] */
	for( i = 1; i < len - 2; ++i )
		if( d[i] == 0x00 && d[i+1] == 0x00 && d[i+2] == 0x01 )
			return i;
	return 0;
}

static int add_to_config( struct rtp_mpeg4 *out, unsigned char *d, int len )
{
	if( out->config_len + len > sizeof( out->config ) )
	{
		spook_log( SL_WARN,
			"rtp-mpeg4: config data is %d bytes too large",
			out->config_len + len - sizeof( out->config ) );
		return -1;
	}
	memcpy( out->config + out->config_len, d, len );
	out->config_len += len;
	return 0;
}

static void finish_config( struct rtp_mpeg4 *out )
{
	int i;
	char *o = out->fmtp;

	o += sprintf( o, "profile-level-id=%d;config=", out->pali );

	for( i = 0; i < out->config_len; ++i )
		o += sprintf( o, "%02X", out->config[i] );

	out->config_len = 0;
	out->init_done = 1;
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

static void parse_visual_object_sequence( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	out->pali = d[4];
	add_to_config( out, d, len );
	add_to_rtp_buf( out, d, len );
}

static void parse_visual_object( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	int visual_object_type;
	int off = 32;

	if( get_field( d, 1, &off ) ) off += 7;
	visual_object_type = get_field( d, 4, &off );
	//spook_log( SL_DEBUG, "visual_object_type = %d", visual_object_type );

	if( out->config_len > 0 ) add_to_config( out, d, len );
	add_to_rtp_buf( out, d, len );
}

static void parse_video_object( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	// should be empty except for the start code

//	if( out->config_len > 0 )
		add_to_config( out, d, len );
	add_to_rtp_buf( out, d, len );
}

static void parse_1b2( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	if( out->config_len > 0 )
		add_to_config( out, d, len );
	add_to_rtp_buf( out, d, len );
}

static void parse_video_object_layer( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	int off = 41, i;

	if( get_field( d, 1, &off ) ) off += 7; // object layer identifier
	if( get_field( d, 4, &off ) == 15 ) // aspect ratio info
		off += 16; // extended par
	if( get_field( d, 1, &off ) ) // vol control parameters
	{
		off += 3; // chroma format, low delay
		if( get_field( d, 1, &off ) ) off += 79; // vbw parameters
	}
	off += 2; // video object layer shape
	if( ! get_field( d, 1, &off ) )
	{
		spook_log( SL_WARN, "rtp-mpeg4: missing marker" );
		return;
	}
	i = out->vop_time_increment_resolution = get_field( d, 16, &off );
	if( ! get_field( d, 1, &off ) )
	{
		spook_log( SL_WARN, "rtp-mpeg4: missing marker" );
		return;
	}
	for( out->vtir_bitlen = 0; i != 0; i >>= 1 ) ++out->vtir_bitlen;
	if( get_field( d, 1, &off ) )
	{
		out->vop_time_increment = get_field( d, out->vtir_bitlen, &off );
		if( out->vop_time_increment == 0 )
		{
			spook_log( SL_WARN,
				"rtp-mpeg4: fixed VOP time increment is 0!" );
			return;
		}
		out->ts_incr = 90000 * out->vop_time_increment / out->vop_time_increment_resolution;
		//spook_log( SL_DEBUG, "apparent framerate: %f",
		//	(double)out->vop_time_increment_resolution
		//		/ (double)out->vop_time_increment );
	} else out->vop_time_increment = 0;
	spook_log( SL_DEBUG, "rtp-mpeg4: vop_time_increment = %d, vop_time_increment_resolution = %d", out->vop_time_increment, out->vop_time_increment_resolution);

	if( out->config_len > 0 ) add_to_config( out, d, len );
	add_to_rtp_buf( out, d, len );
}

static void parse_video_object_plane( struct rtp_mpeg4 *out,
				unsigned char *d, int len )
{
	int off = 32;
	int vop_coding_type, modulo_time_base, var_time_increment;

	if( out->config_len > 0 ) finish_config( out );

	if( ! out->init_done ) return;

	vop_coding_type = get_field( d, 2, &off );
	//spook_log( SL_DEBUG, "vop coding type is %d", vop_coding_type );
	for( modulo_time_base = 0; off / 8 < len; ++modulo_time_base )
		if( ! get_field( d, 1, &off ) ) break;
	if( ! get_field( d, 1, &off ) )
	{
		spook_log( SL_WARN, "rtp-mpeg4: missing marker!" );
		return;
	}
	var_time_increment = get_field( d, out->vtir_bitlen, &off );

	out->timestamp += out->ts_incr;

	if( vop_coding_type == 0 )
		spook_log( SL_DEBUG,
			"frame type %c, size %d, vop time %d, modulo %d",
			"IPBS"[vop_coding_type], len,
			var_time_increment, modulo_time_base );

	add_to_rtp_buf( out, d, len );
}

static int parse_mpeg4_frame( struct rtp_mpeg4 *out, unsigned char *d, int len )
{
	unsigned int start_code;

	start_code = GET_32( d );
	//spook_log( SL_DEBUG, "frame start code: %08x", start_code );
	if( start_code == 0x000001B0 )
		parse_visual_object_sequence( out, d, len );
	else if( start_code == 0x000001B2 )
		parse_1b2( out, d, len );
	else if( start_code == 0x000001B5 )
		parse_visual_object( out, d, len );
	else if( start_code >= 0x00000100 && start_code <= 0x0000011F )
		parse_video_object( out, d, len );
	else if( start_code >= 0x00000120 && start_code <= 0x0000012F )
		parse_video_object_layer( out, d, len );
	else if( start_code == 0x000001B6 )
	{
		parse_video_object_plane( out, d, len );
		return 1;
	}
	return 0;
}

static int mpeg4_process_frame( struct frame *f, void *d )
{
	struct rtp_mpeg4 *out = (struct rtp_mpeg4 *)d;
	int flen, start;

	out->iov_count = 0;
	for( start = 0; start < f->length; start += flen )
	{
		flen = find_next_code( f->d + start, f->length - start );
		if( flen == 0 ) flen = f->length - start;
		parse_mpeg4_frame( out, f->d + start, flen );
	}
	return out->init_done;
}

static int mpeg4_get_sdp( char *dest, int len, int payload, int port, void *d )
{
	struct rtp_mpeg4 *out = (struct rtp_mpeg4 *)d;

	if( ! out->init_done ) return -1;
	return snprintf( dest, len, "m=video %d RTP/AVP %d\r\na=rtpmap:%d MP4V-ES/90000\r\na=fmtp:%d %s\r\n", port, payload, payload, payload, out->fmtp );
}

static int mpeg4_send( struct rtp_endpoint *ep, void *d )
{
	struct rtp_mpeg4 *out = (struct rtp_mpeg4 *)d;
	int i, j, space, off;
	struct iovec v[32];

	i = 0;
	j = 1;
	off = 0;
	space = ep->max_data_size;
	while( i < out->iov_count )
	{
		v[j].iov_base = out->iov[i].iov_base + off;
		v[j].iov_len = out->iov[i].iov_len - off;
		if( space < v[j].iov_len )
		{
			v[j].iov_len = space;
			++j;
			if( send_rtp_packet( ep, v, j, out->timestamp, 0 ) < 0 )
				return -1;
			off += space;
			space = ep->max_data_size;
			j = 1;
		} else
		{
			space -= v[j].iov_len;
			++j;
			++i;
			off = 0;
		}
	}
	if( send_rtp_packet( ep, v, j, out->timestamp, 1 ) < 0 ) return -1;
	return 0;
}

struct rtp_media *new_rtp_media_mpeg4(void)
{
	struct rtp_mpeg4 *out;

	out = (struct rtp_mpeg4 *)malloc( sizeof( struct rtp_mpeg4 ) );
	out->iov_count = 0;
	out->config_len = 0;
	out->init_done = 0;
	out->pali = 0;
	out->timestamp = 0;

	return new_rtp_media( mpeg4_get_sdp, NULL, mpeg4_process_frame,
			mpeg4_send, out );
}
