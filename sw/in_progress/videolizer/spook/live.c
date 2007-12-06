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

struct live_source;

struct live_session {
	struct live_session *next;
	struct live_session *prev;
	struct session *sess;
	struct live_source *source;
	int playing;
};

struct live_track {
	int index;
	struct live_source *source;
	struct stream_destination *stream;
	int ready;
	struct rtp_media *rtp;
};

struct live_source {
	struct live_session *sess_list;
	struct live_track track[MAX_TRACKS];
};

static int live_get_sdp( struct session *s, char *dest, int *len,
				char *path )
{
	struct live_session *ls = (struct live_session *)s->private;
	int i = 0, t;
	char *addr = "IP4 0.0.0.0";

	if( ! ls->source->track[0].rtp || ! ls->source->track[0].ready )
		return 0;

	if( s->ep[0] && s->ep[0]->trans_type == RTP_TRANS_UDP )
		addr = s->ep[0]->trans.udp.sdp_addr;

	i = snprintf( dest, *len,
		"v=0\r\no=- 1 1 IN IP4 127.0.0.1\r\ns=Test\r\na=type:broadcast\r\nt=0 0\r\nc=IN %s\r\n", addr );

	for( t = 0; t < MAX_TRACKS && ls->source->track[t].rtp; ++t )
	{
		int port;

		if( ! ls->source->track[t].ready ) return 0;

		if( s->ep[t] && s->ep[t]->trans_type == RTP_TRANS_UDP )
			port = s->ep[t]->trans.udp.sdp_port;
		else
			port = 0;

		i += ls->source->track[t].rtp->get_sdp( dest + i, *len - i,
				96 + t, port,
				ls->source->track[t].rtp->private );
		if( port == 0 ) // XXX What's a better way to do this?
			i += sprintf( dest + i, "a=control:track%d\r\n", t );
	}

	*len = i;
	return t;
}

static int live_setup( struct session *s, int t )
{
	struct live_session *ls = (struct live_session *)s->private;
	int payload = 96 + t;

	if( ! ls->source->track[t].rtp ) return -1;

	if( ls->source->track[t].rtp->get_payload )
		payload = ls->source->track[t].rtp->get_payload( payload,
					ls->source->track[t].rtp->private );
	s->ep[t] = new_rtp_endpoint( payload );
	s->ep[t]->session = s;
	
	return 0;
}

static void live_play( struct session *s, double *start )
{
	struct live_session *ls = (struct live_session *)s->private;
	int t;

	if( start ) *start = -1;
	ls->playing = 1;
	for( t = 0; t < MAX_TRACKS && ls->source->track[t].rtp; ++t )
		if( s->ep[t] ) set_waiting( ls->source->track[t].stream, 1 );
}

static void track_check_running( struct live_source *source, int t )
{
	struct live_session *ls;

	for( ls = source->sess_list; ls; ls = ls->next )
		if( ls->playing && ls->sess->ep[t] ) return;

	set_waiting( source->track[t].stream, 0 );
}

static void live_teardown( struct session *s, struct rtp_endpoint *ep )
{
	struct live_session *ls = (struct live_session *)s->private;
	int i, remaining = 0;

	for( i = 0; i < MAX_TRACKS && ls->source->track[i].rtp; ++i )
	{
		if( ! s->ep[i] ) continue;
		if( ! ep || s->ep[i] == ep )
		{
			del_rtp_endpoint( s->ep[i] );
			s->ep[i] = NULL;
			track_check_running( ls->source, i );
		} else ++remaining;
	}

	if( remaining == 0 )
	{
		if( ls->next ) ls->next->prev = ls->prev;
		if( ls->prev ) ls->prev->next = ls->next;
		else ls->source->sess_list = ls->next;
		free( ls );
		del_session( s );
	}
}

static struct session *live_open( char *path, void *d )
{
	struct live_source *source = (struct live_source *)d;
	struct live_session *ls;

	ls = (struct live_session *)malloc( sizeof( struct live_session ) );
	ls->next = source->sess_list;
	if( ls->next ) ls->next->prev = ls;
	source->sess_list = ls;
	ls->prev = NULL;
	ls->sess = new_session();
	ls->source = source;
	ls->playing = 0;
	ls->sess->get_sdp = live_get_sdp;
	ls->sess->setup = live_setup;
	ls->sess->play = live_play;
	ls->sess->teardown = live_teardown;
	ls->sess->private = ls;

	return ls->sess;
}

static void next_live_frame( struct frame *f, void *d )
{
	struct live_track *track = (struct live_track *)d;
	struct live_session *ls, *next;

	if( ! track->rtp->frame( f, track->rtp->private ) )
	{
		unref_frame( f );
		return;
	}

	if( ! track->ready )
	{
		set_waiting( track->stream, 0 );
		track->ready = 1;
	}

	for( ls = track->source->sess_list; ls; ls = next )
	{
		next = ls->next;
		if( ls->playing && ls->sess->ep[track->index] )
			track->rtp->send( ls->sess->ep[track->index],
						track->rtp->private );
	}

	unref_frame( f );
}

/************************ CONFIGURATION DIRECTIVES ************************/

static void *start_block(void)
{
	struct live_source *source;
	int i;

	source = (struct live_source *)malloc( sizeof( struct live_source ) );
	source->sess_list = NULL;
	for( i = 0; i < MAX_TRACKS; ++i )
	{
		source->track[i].index = i;
		source->track[i].source = source;
		source->track[i].stream = NULL;
		source->track[i].ready = 0;
		source->track[i].rtp = NULL;
	}

	return source;
}

static int end_block( void *d )
{
	struct live_source *source = (struct live_source *)d;

	if( ! source->track[0].rtp )
	{
		spook_log( SL_ERR, "live: no media sources specified!" );
		return -1;
	}

	return 0;
}

static int set_track( int num_tokens, struct token *tokens, void *d )
{
	struct live_source *source = (struct live_source *)d;
	int t, formats[] = { FORMAT_MPEG4, FORMAT_MPV, FORMAT_H263, FORMAT_JPEG,
				FORMAT_PCM, FORMAT_ALAW, FORMAT_MPA };

	for( t = 0; t < MAX_TRACKS && source->track[t].rtp; ++t );

	if( t == MAX_TRACKS )
	{
		spook_log( SL_ERR, "live: exceeded maximum number of tracks" );
		return -1;
	}

	if( ! ( source->track[t].stream = connect_to_stream( tokens[1].v.str,
			next_live_frame, &source->track[t], formats, 7 ) ) )
	{
		spook_log( SL_ERR,
				"live: unable to connect to stream \"%s\"",
				tokens[1].v.str );
		return -1;
	}

	switch( source->track[t].stream->stream->format )
	{
	case FORMAT_MPEG4:
		source->track[t].rtp = new_rtp_media_mpeg4();
		break;
	case FORMAT_MPV:
		source->track[t].rtp = new_rtp_media_mpv();
		break;
	case FORMAT_H263:
		source->track[t].rtp = new_rtp_media_h263_stream(
					source->track[t].stream->stream );
		break;
	case FORMAT_JPEG:
		source->track[t].rtp = new_rtp_media_jpeg_stream(
					source->track[t].stream->stream );
		break;
	case FORMAT_PCM:
	case FORMAT_ALAW:
		source->track[t].rtp = new_rtp_media_rawaudio_stream(
					source->track[t].stream->stream );
		break;
	case FORMAT_MPA:
		source->track[t].rtp = new_rtp_media_mpa();
		break;
	}

	if( ! source->track[t].rtp ) return -1;

	set_waiting( source->track[t].stream, 1 );

	return 0;
}

static int set_path( int num_tokens, struct token *tokens, void *d )
{
	if( num_tokens == 2 )
	{
		new_rtsp_location( tokens[1].v.str, NULL, NULL, NULL,
				live_open, d );
		return 0;
	}
	if( num_tokens == 5 )
	{
		new_rtsp_location( tokens[1].v.str, tokens[2].v.str,
				tokens[3].v.str, tokens[4].v.str,
				live_open, d );
		return 0;
	}
	spook_log( SL_ERR, "rtsp-handler: syntax: Path <path> [<realm> <username> <password>]" );
	return -1;
}

#if 0
static int set_sip_line( int num_tokens, struct token *tokens, void *d )
{
	new_sip_line( tokens[1].v.str, live_open, d );
	return 0;
}
#endif

static struct statement config_statements[] = {
	/* directive name, process function, min args, max args, arg types */
	{ "track", set_track, 1, 1, { TOKEN_STR } },
	{ "path", set_path, 1, 4, { TOKEN_STR, TOKEN_STR, TOKEN_STR, TOKEN_STR } },
	/* { "sipline", set_sip_line, 1, 1, { TOKEN_STR } }, */

	/* empty terminator -- do not remove */
	{ NULL, NULL, 0, 0, {} }
};

int live_init(void)
{
	register_config_context( "rtsp-handler", "live", start_block, end_block,
					config_statements );
	return 0;
}
