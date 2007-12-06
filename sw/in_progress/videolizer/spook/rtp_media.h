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

typedef int (*rtp_media_get_sdp_func)( char *dest, int len, int payload,
						int port, void *d );
typedef int (*rtp_media_get_payload_func)( int payload, void *d );
typedef int (*rtp_media_frame_func)( struct frame *f, void *d );
typedef int (*rtp_media_send_func)( struct rtp_endpoint *ep, void *d );

struct rtp_media {
	rtp_media_get_sdp_func get_sdp;
	rtp_media_get_payload_func get_payload;
	rtp_media_frame_func frame;
	rtp_media_send_func send;
	void *private;
};

struct rtp_media *new_rtp_media( rtp_media_get_sdp_func get_sdp,
	rtp_media_get_payload_func get_payload, rtp_media_frame_func frame,
	rtp_media_send_func send, void *private );
struct rtp_media *new_rtp_media_mpeg4(void);
struct rtp_media *new_rtp_media_mpv(void);
struct rtp_media *new_rtp_media_h263_stream( struct stream *stream );
struct rtp_media *new_rtp_media_jpeg_stream( struct stream *stream );
struct rtp_media *new_rtp_media_mpa(void);
struct rtp_media *new_rtp_media_rawaudio_stream( struct stream *stream );
