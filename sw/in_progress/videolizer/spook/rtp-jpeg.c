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
#include <jpeg_tables.h>

struct rtp_jpeg {
	int type;
	int q;
	int width;
	int height;
	int luma_table;
	int chroma_table;
	unsigned char *quant[16];
	unsigned char *scan_data;
	int scan_data_len;
	int init_done;
	int ts_incr;
	unsigned int timestamp;
};

/* Unused at present */
static int guess_q( unsigned char lq[] )
{
	int s = 0, i, count = 0;

	s = 100 * lq[0] / jpeg_luma_quantizer[0];
#if 0
	printf( "Guessed quant scale is %d:\n", s );
	for( i = 0; i < 8; ++i )
	{
		printf( "  " );
		for( j = 0; j < 8; ++j )
			printf( "%3d ", lq[8*i+j] );
		printf( "  " );
		for( j = 0; j < 8; ++j )
			printf( "%1.2f ", (double)lq[8*i+j] /
					(double)jpeg_luma_quantizer[8*i+j] );
		printf( "\n" );
	}
#endif
	if( s > 100 ) return 5000 / s;
	else return 100 - ( s >> 1 );
}

static int parse_DQT( struct rtp_jpeg *out, unsigned char *d, int len )
{
	int i;

	for( i = 0; i < len; i += 65 )
	{
		if( ( d[i] & 0xF0 ) != 0 )
		{
			printf( "Unsupported quant table precision 0x%X!\n",
					d[i] & 0xF0 );
			return -1;
		}
		out->quant[d[i] & 0xF] = d + i + 1;
	}
	return 0;
}

static int parse_SOF( struct rtp_jpeg *out, unsigned char *d, int len )
{
	int c;

	out->chroma_table = -1;
	if( d[0] != 8 )
	{
		printf( "Invalid precision %d in SOF0\n", d[0] );
		return -1;
	}
	out->height = GET_16( d + 1 );
	out->width = GET_16( d + 3 );
	if( ( out->height & 0x7 ) || ( out->width & 0x7 ) )
	{
		printf( "Width/height not divisible by 8!\n" );
		return -1;
	}
	out->width >>= 3;
	out->height >>= 3;
	if( d[5] != 3 )
	{
		printf( "Number of components is %d, not 3!\n", d[5] );
		return -1;
	}
	/* Loop over the parameters for each component */
	for( c = 6; c < 6 + 3 * 3; c += 3 )
	{
		if( d[c + 2] >= 16 || ! out->quant[d[c + 2]] )
		{
			printf( "Component %d specified undefined quant table %d!\n", d[c], d[c + 2] );
			return -1;
		}
		switch( d[c] ) /* d[c] contains the component ID */
		{
		case 1: /* Y */
			if( d[c + 1] == 0x11 ) out->type = 0;
			else if( d[c + 1] == 0x22 ) out->type = 1;
			else
			{
				printf( "Invalid sampling factor 0x%02X in Y component!\n", d[c + 1] );
				return -1;
			}
			out->luma_table = d[c + 2];
			break;
		case 2: /* Cb */
		case 3: /* Cr */
			if( d[c + 1] != 0x11 )
			{
				printf( "Invalid sampling factor 0x%02X in %s component!\n", d[c + 1], d[c] == 2 ? "Cb" : "Cr" );
				return -1;
			}
			if( out->chroma_table < 0 )
				out->chroma_table = d[c + 2];
			else if( out->chroma_table != d[c + 2] )
			{
				printf( "Cb and Cr components do not share a quantization table!\n" );
				return -1;
			}
			break;
		default:
			printf( "Invalid component %d in SOF!\n", d[c] );
			return -1;
		}
	}
	return 0;
}

static int parse_DHT( struct rtp_jpeg *out, unsigned char *d, int len )
{
	/* We should verify that this coder uses the standard Huffman tables */
	return 0;
}

static int jpeg_process_frame( struct frame *f, void *d )
{
	struct rtp_jpeg *out = (struct rtp_jpeg *)d;
	int i, blen;

	for( i = 0; i < 16; ++i ) out->quant[i] = NULL;

	for( i = 0; i < f->length; i += blen + 2 )
	{
		if( f->d[i] != 0xFF )
		{
			printf( "Found %02X at %d, expecting FF\n", f->d[i], i );
			out->scan_data_len = 0;
			return 0;
		}

		/* SOI (FF D8) is a bare marker with no length field */
		if( f->d[i + 1] == 0xD8 ) blen = 0;
		else blen = GET_16( f->d + i + 2 );

		switch( f->d[i + 1] )
		{
		case 0xDB: /* Quantization Table */
			if( parse_DQT( out, f->d + i + 4, blen - 2 ) < 0 )
			{
				out->scan_data_len = 0;
				return 0;
			}
			break;
		case 0xC0: /* Start of Frame */
			if( parse_SOF( out, f->d + i + 4, blen - 2 ) < 0 )
			{
				out->scan_data_len = 0;
				return 0;
			}
			break;
		case 0xC4: /* Huffman Table */
			if( out->init_done ) break; /* only parse DHT once */
			if( parse_DHT( out, f->d + i + 4, blen - 2 ) < 0 )
			{
				out->scan_data_len = 0;
				return 0;
			}
			out->init_done = 1;
			break;
		case 0xDA: /* Start of Scan */
			out->scan_data = f->d + i + 2 + blen;
			out->scan_data_len = f->length - i - 2 - blen;
			out->timestamp += out->ts_incr;
			return out->init_done;
		}
	}

	printf( "Found no scan data!\n" );
	out->scan_data_len = 0;
	return 0;
}

static int jpeg_get_sdp( char *dest, int len, int payload, int port, void *d )
{
	return snprintf( dest, len, "m=video %d RTP/AVP 26\r\n", port );
}

static int jpeg_get_payload( int payload, void *d )
{
	return 26;
}

static int jpeg_send( struct rtp_endpoint *ep, void *d )
{
	struct rtp_jpeg *out = (struct rtp_jpeg *)d;
	int i, plen, vcnt, hdr_len;
	struct iovec v[6];
	unsigned char vhdr[8], qhdr[4];

	/* Main JPEG header */
	vhdr[0] = 0; /* type-specific, value 0 -> non-interlaced frame */
	/* vhdr[1..3] is the fragment offset */
	vhdr[4] = out->type; /* type */
	vhdr[5] = 255; /* Q, value 255 -> dynamic quant tables */
	vhdr[6] = out->width;
	vhdr[7] = out->height;
	v[1].iov_base = vhdr;
	v[1].iov_len = 8;

	/* Quantization table header */
	qhdr[0] = 0; /* MBZ */
	qhdr[1] = 0; /* precision */
	PUT_16( qhdr + 2, 2 * 64 ); /* length */
	v[2].iov_base = qhdr;
	v[2].iov_len = 4;

	/* Luma quant table */
	v[3].iov_base = out->quant[out->luma_table];
	v[3].iov_len = 64;

	/* Chroma quant table */
	v[4].iov_base = out->quant[out->chroma_table];
	v[4].iov_len = 64;

	hdr_len = 140;
	vcnt = 5;

	for( i = 0; i < out->scan_data_len; i += plen )
	{
		plen = out->scan_data_len - i;
		if( plen > ep->max_data_size - hdr_len )
			plen = ep->max_data_size - hdr_len;
		/* No hay PUT_24 macro... */
		vhdr[1] = i >> 16;
		vhdr[2] = ( i >> 8 ) & 0xff;
		vhdr[3] = i & 0xff;
		v[vcnt].iov_base = out->scan_data + i;
		v[vcnt].iov_len = plen;
		if( send_rtp_packet( ep, v, vcnt + 1, out->timestamp,
					plen + i == out->scan_data_len ) < 0 )
			return -1;
		/* Done with all headers except main JPEG header */
		vcnt = 2;
		hdr_len = 8;
	}
	return 0;
}

struct rtp_media *new_rtp_media_jpeg_stream( struct stream *stream )
{
	struct rtp_jpeg *out;
	int fincr, fbase;

	stream->get_framerate( stream, &fincr, &fbase );
	out = (struct rtp_jpeg *)malloc( sizeof( struct rtp_jpeg ) );
	out->init_done = 0;
	out->timestamp = 0;
	out->ts_incr = 90000 * fincr / fbase;

	return new_rtp_media( jpeg_get_sdp, jpeg_get_payload,
					jpeg_process_frame, jpeg_send, out );
}
